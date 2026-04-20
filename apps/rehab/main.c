#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

#include "pico_uart_transports.h"
#include "debug_uart.h"
#include <math.h>

// Configurazione Pin
#define I2C_PORT i2c0

const uint PIN_LED_TREMOR = 16;
const uint PIN_SDA = 4;
const uint PIN_SCL = 5;
const uint LED_PIN = 25; // LED integrato del Pico
const uint PIN_JOY_X = 26;
const uint PIN_JOY_Y = 27;
const uint PIN_BTN_START = 21;
const uint PIN_BUZZER = 15;


// --- STRUTTURE DATI ---

// 1. Pacchetto per la CODA (Trasporto dati verso ROS)
// Garantisce la coerenza temporale: ogni pacchetto è un'istantanea atomica.
typedef struct {
    float x;
    float y;
    float tremor;
    bool active; 
} SensorPacket_t;

// 2. Stato Globale del Sistema (Condiviso via Mutex)
// Serve per il controllo degli attuatori (LED, Buzzer) in tempo reale.
struct SystemState {
    bool active;
    float current_tremor_intensity;
};

// --- RISORSE DI CONCORRENZA ---
struct SystemState global_state = {false, 0.0f};
bool collision_alarm = false;
uint32_t last_ros_message_time = 0;
const uint32_t ROS_WATCHDOG_TIMEOUT = 1000;

SemaphoreHandle_t xMutex;
QueueHandle_t xSensorQueue;

TaskHandle_t xHandleROS = NULL;
TaskHandle_t xHandleHeartbeat = NULL;
TaskHandle_t xHandleTremor = NULL;
TaskHandle_t xHandleInput = NULL;
TaskHandle_t xHandleBuzzer = NULL;

// Variabili globali per micro-ROS
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Bool msg_sub;
geometry_msgs__msg__Twist msg_pub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- INIZIALIZZAZIONE MPU6050 ---
void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00}; // Sveglia il sensore
    i2c_write_blocking(I2C_PORT, 0x68, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3]) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, 0x68, &val, 1, true);
    i2c_read_blocking(I2C_PORT, 0x68, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[i * 2 + 1]);
    }
}

// Callback: viene eseguita ogni volta che il PC invia un messaggio su /collision_alert
void subscription_callback(const void * msin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msin;
    collision_alarm = msg->data;
    last_ros_message_time = to_ms_since_boot(get_absolute_time()); // Registra quando è arrivato
}

// --- TASK 1: GESTIONE INPUT (Alta Priorità) ---
void Task_Input(void *pvParameters) {
    bool last_btn_state = true;
    const float DEADZONE = 0.05f;
    SensorPacket_t packet;

    for (;;) {
        // Lettura Joystick
        adc_select_input(0);
        float raw_x = ((float)adc_read() - 2048.0f) / 2048.0f;
        adc_select_input(1);
        float raw_y = ((float)adc_read() - 2048.0f) / 2048.0f;

        if (fabs(raw_x) < DEADZONE) raw_x = 0.0f;
        if (fabs(raw_y) < DEADZONE) raw_y = 0.0f;

        // Gestione Pulsante con Debounce
        bool current_btn_state = gpio_get(PIN_BTN_START);
        if (last_btn_state && !current_btn_state) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (!gpio_get(PIN_BTN_START)) {
                xSemaphoreTake(xMutex, portMAX_DELAY);
                global_state.active = !global_state.active;
                xSemaphoreGive(xMutex);
            }
        }
        last_btn_state = current_btn_state;

        // Creazione "Istantanea" per ROS (Encapsulation)
        xSemaphoreTake(xMutex, portMAX_DELAY);
        packet.x = raw_x;
        packet.y = raw_y;
        packet.tremor = global_state.current_tremor_intensity;
        packet.active = global_state.active;
        xSemaphoreGive(xMutex);

        // Invio alla Coda (Sovrascrittura = vogliamo solo l'ultimo dato fresco)
        xQueueOverwrite(xSensorQueue, &packet);

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// --- TASK 2: MICRO-ROS (Media Priorità) ---
void Task_ROS(void *pvParameters) {
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_rehab_node", "", &support);

    rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/surgeon_input");
    rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/collision_alert");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);

    SensorPacket_t data_from_queue;

    for (;;) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Watchdog di Connessione
        if (now - last_ros_message_time > ROS_WATCHDOG_TIMEOUT) {
            xSemaphoreTake(xMutex, portMAX_DELAY);
            if (global_state.active) {
                global_state.active = false;
                collision_alarm = false;
            }
            xSemaphoreGive(xMutex);
        }

        // Consumiamo il pacchetto dalla coda
        if (xQueueReceive(xSensorQueue, &data_from_queue, 0) == pdPASS) {
            msg_pub.linear.x = data_from_queue.x;
            msg_pub.linear.y = data_from_queue.y;
            msg_pub.angular.x = data_from_queue.tremor;
            msg_pub.angular.z = data_from_queue.active ? 1.0f : 0.0f;
            
            rcl_publish(&publisher, &msg_pub, NULL);
        }

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(30)); 
    }
}

// --- TASK 3: FEEDBACK ACUSTICO (Priorità Massima) ---
void Task_Buzzer(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        bool run = global_state.active && collision_alarm;
        xSemaphoreGive(xMutex);

        if (run) {
            gpio_put(PIN_BUZZER, 1); vTaskDelay(pdMS_TO_TICKS(80));
            gpio_put(PIN_BUZZER, 0); vTaskDelay(pdMS_TO_TICKS(80));
        } else {
            gpio_put(PIN_BUZZER, 0); vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// --- TASK 4: ANALISI DEL TREMORE (Alta Priorità) ---
void Task_Tremor(void *pvParameters) {
    int16_t accel[3];
    float samples[20] = {0};
    int idx = 0;

    for (;;) {
        mpu6050_read_raw(accel);
        float mag = sqrt(pow(accel[0]/16384.0f, 2) + pow(accel[1]/16384.0f, 2) + pow(accel[2]/16384.0f, 2));
        samples[idx] = mag;
        idx = (idx + 1) % 20;

        float mean = 0, var = 0;
        for(int i=0; i<20; i++) mean += samples[i];
        mean /= 20.0f;
        for(int i=0; i<20; i++) var += pow(samples[i] - mean, 2);
        float intensity = (var / 20.0f) * 100.0f;

        xSemaphoreTake(xMutex, portMAX_DELAY);
        if (global_state.active) {
            global_state.current_tremor_intensity = intensity;
            gpio_put(PIN_LED_TREMOR, intensity > 0.7f);
        } else {
            global_state.current_tremor_intensity = 0.0f;
            gpio_put(PIN_LED_TREMOR, 0);
        }
        xSemaphoreGive(xMutex);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- TASK 5: HEARTBEAT LED (Priorità Minima) ---
void Task_Heartbeat(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        bool active = global_state.active;;
        xSemaphoreGive(xMutex);

        gpio_put(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(active ? 500 : 100));
        gpio_put(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(active ? 500 : 900));
    }
}

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(PIN_JOY_X);
    adc_gpio_init(PIN_JOY_Y);

    gpio_init(PIN_BTN_START);
    gpio_set_dir(PIN_BTN_START, GPIO_IN);
    gpio_pull_up(PIN_BTN_START);

    gpio_init(PIN_BUZZER);
    gpio_set_dir(PIN_BUZZER, GPIO_OUT);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    mpu6050_reset();

    gpio_init(PIN_LED_TREMOR);
    gpio_set_dir(PIN_LED_TREMOR, GPIO_OUT);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    xMutex = xSemaphoreCreateMutex();
    xSensorQueue = xQueueCreate(1, sizeof(SensorPacket_t));

    xTaskCreate(Task_Tremor, "Tremor", 512, NULL, 4, xHandleTremor);
    xTaskCreate(Task_Buzzer, "Buzzer", 256, NULL, 4, xHandleBuzzer);
    xTaskCreate(Task_Input, "Input", 256, NULL, 3, xHandleInput);
    xTaskCreate(Task_ROS, "ROS", 2048, NULL, 2, xHandleROS);
    xTaskCreate(Task_Heartbeat, "Heartbeat", 128, NULL, 1, xHandleHeartbeat);

    vTaskCoreAffinitySet(xHandleROS, (1 << 0));
    vTaskCoreAffinitySet(xHandleHeartbeat, (1 << 0));
    vTaskCoreAffinitySet(xHandleTremor, (1 << 1));
    vTaskCoreAffinitySet(xHandleInput, (1 << 1));
    vTaskCoreAffinitySet(xHandleBuzzer, (1 << 1));

    vTaskStartScheduler();
}