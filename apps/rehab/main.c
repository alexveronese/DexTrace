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

// Risorse Condivise e Concorrenza
bool collision_alarm = false;
uint32_t last_ros_message_time = 0;
const uint32_t ROS_WATCHDOG_TIMEOUT = 1000;

struct SystemState {
    float x, y;
    float tremor_intensity;
    bool active;
};

struct SystemState state = {0.0f, 0.0f, false};
SemaphoreHandle_t xMutex;

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
    
    float center_x = 2048.0f;
    float center_y = 2048.0f;
    
    // Imposta la tua Deadzone collaudata al 5%
    const float DEADZONE = 0.05f; 

    for (;;) {
        // Leggi ADC (Joystick)
        adc_select_input(0);
        // Normalizziamo usando il VERO centro calcolato, non più 2048 fisso
        float raw_x = (adc_read() - center_x)/center_x; 
        
        adc_select_input(1);
        float raw_y = (adc_read() - center_y)/center_y;

        // --- APPLICAZIONE DEADZONE ---
        if (fabs(raw_x) < DEADZONE) {
            raw_x = 0.0f;
        }
        if (fabs(raw_y) < DEADZONE) {
            raw_y = 0.0f;
        }

        // Gestione Pulsante Start/Stop (Debounce)
        bool current_btn_state = gpio_get(PIN_BTN_START);
        if (last_btn_state && !current_btn_state) { 
            vTaskDelay(pdMS_TO_TICKS(50));
            if (!gpio_get(PIN_BTN_START)) {
                xSemaphoreTake(xMutex, portMAX_DELAY);
                state.active = !state.active; 
                xSemaphoreGive(xMutex);
            }
        }
        last_btn_state = current_btn_state;

        // Aggiorna coordinate nella risorsa condivisa
        xSemaphoreTake(xMutex, portMAX_DELAY);
        state.x = raw_x;
        state.y = raw_y;
        xSemaphoreGive(xMutex);

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz costanti
    }
}

// --- TASK 2: MICRO-ROS (Media Priorità) ---
void Task_ROS(void *pvParameters) {
    // 1. Setup Trasporto e Allocatore
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);
    allocator = rcl_get_default_allocator();

    // 2. Inizializzazione Support e Node
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_rehab_node", "", &support);

    // 3. Inizializzazione Publisher (Joystick -> PC)
    rclc_publisher_init_default(&publisher, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/surgeon_input");

    // 4. Inizializzazione Subscriber (PC -> Buzzer)
    rclc_subscription_init_default(&subscriber, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/collision_alert");

    // 5. Setup Executor: deve gestire 1 comunicazione in ingresso (il subscriber)
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);

    for (;;) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // --- LOGICA DI RESET AUTOMATICO (WATCHDOG) ---
        // Se l'ultimo messaggio dal PC è più vecchio del timeout
        if (current_time - last_ros_message_time > ROS_WATCHDOG_TIMEOUT) {
            xSemaphoreTake(xMutex, portMAX_DELAY);
            if (state.active) {
                state.active = false; // Forza il reset dello stato
                collision_alarm = false; // Spegne l'allarme
                printf("Connessione PC persa: Reset sistema in Standby.\n");
            }
            xSemaphoreGive(xMutex);
        }
        
        // Lettura protetta dallo stato per il publishing
        xSemaphoreTake(xMutex, portMAX_DELAY);
        msg_pub.linear.x = state.x;
        msg_pub.linear.y = state.y;
        msg_pub.angular.z = state.active ? 1.0 : 0.0;
        msg_pub.angular.x = state.tremor_intensity;
        xSemaphoreGive(xMutex);

        // Pubblichiamo la posizione dell'utente
        rcl_publish(&publisher, &msg_pub, NULL);

        // Controlliamo se sono arrivati messaggi dal PC (l'allarme)
        // timeout_ns = 0 significa "controlla e vai avanti senza bloccarti"
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        vTaskDelay(pdMS_TO_TICKS(30)); // Frequenza di rete (~33Hz)
    }
}

// --- TASK 3: FEEDBACK ACUSTICO (Priorità Massima) ---
void Task_Buzzer(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        bool is_active = state.active;
        bool alarm = collision_alarm;
        xSemaphoreGive(xMutex);

        if (is_active && alarm) {
            gpio_put(PIN_BUZZER, 1);
            vTaskDelay(pdMS_TO_TICKS(80));
            gpio_put(PIN_BUZZER, 0);
            vTaskDelay(pdMS_TO_TICKS(80));
        } else {
            gpio_put(PIN_BUZZER, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// --- TASK 4: ANALISI DEL TREMORE (Alta Priorità) ---
void Task_Tremor(void *pvParameters) {
    int16_t accel[3];
    float samples[20] = {0};
    int sample_idx = 0;
    const float TREMOR_THRESHOLD = 0.7f; // La tua soglia ottimizzata

    for (;;) {
        mpu6050_read_raw(accel);
        
        float ax = accel[0] / 16384.0f;
        float ay = accel[1] / 16384.0f;
        float az = accel[2] / 16384.0f;
        float mag = sqrt(ax*ax + ay*ay + az*az);

        samples[sample_idx] = mag;
        sample_idx = (sample_idx + 1) % 20;

        // Calcolo varianza (intensità tremore)
        float mean = 0;
        for(int i=0; i<20; i++) mean += samples[i];
        mean /= 20.0f;

        float var = 0;
        for(int i=0; i<20; i++) var += pow(samples[i] - mean, 2);
        var /= 20.0f;
        float current_intensity = var * 100.0f;

        // --- SINCRONIZZAZIONE CON LO STATO ATTIVO ---
        xSemaphoreTake(xMutex, portMAX_DELAY);
        bool is_active = state.active; 
        
        if (is_active) {
            state.tremor_intensity = current_intensity;
            bool is_shaking = (state.tremor_intensity > TREMOR_THRESHOLD);
            gpio_put(PIN_LED_TREMOR, is_shaking);
        } else {
            // Se il sistema è in STOP, forziamo tutto a zero
            state.tremor_intensity = 0.0f;
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
        bool is_active = state.active;
        xSemaphoreGive(xMutex);

        if (!is_active) {
            // Blink lento (Ready state): 100ms ON, 900ms OFF
            gpio_put(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_put(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(900));
        } else {
            // Luce fissa (In Exercise): Il sistema sta registrando
            gpio_put(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
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

    xTaskCreate(Task_Tremor, "Tremor", 512, NULL, 4, NULL);
    xTaskCreate(Task_Buzzer, "Buzzer", 256, NULL, 4, NULL);
    xTaskCreate(Task_Input, "Input", 256, NULL, 3, NULL);
    xTaskCreate(Task_ROS, "ROS", 2048, NULL, 2, NULL);
    xTaskCreate(Task_Heartbeat, "Heartbeat", 128, NULL, 1, NULL);

    vTaskStartScheduler();
}