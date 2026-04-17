# Define the executable for the Pico Micro-ROS example
add_executable(rehab
    main.c
)

# Link against Pico, FreeRTOS, microros, and shared utilities
target_link_libraries(rehab PRIVATE
    pico_stdlib
    common_utils
    hardware_uart
    hardware_adc
    hardware_i2c
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
    microros
)

# Configure Pico IO settings
pico_enable_stdio_usb(rehab 1)
pico_enable_stdio_uart(rehab 0)

# Compile definitions for Pico config
target_compile_definitions(rehab PRIVATE
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)

# Set UF2 output directory
set_target_properties(rehab PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Generate UF2 and binary outputs
pico_add_extra_outputs(rehab)
