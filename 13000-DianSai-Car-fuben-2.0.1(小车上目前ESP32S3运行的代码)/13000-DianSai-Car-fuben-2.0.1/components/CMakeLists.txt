set(src_dirs
            1-LED
            2-Key
            3-OLED
            4-GPTIMER
            5-System_Timer
            6-PWM
            7-PWMServoMotor
            8-ADC
            9-UART
            10-IIC
            11-WIFI
            12-Motor
            13-Steering_Engine
            14-Encoder
            15-Pid
            16-Infrared_Sensor
            17-ESPNOW
            18-Odometry)

set(include_dirs
            1-LED
            2-Key
            3-OLED
            4-GPTIMER
            5-System_Timer
            6-PWM
            7-PWMServoMotor
            8-ADC
            9-UART
            10-IIC
            11-WIFI
            12-Motor
            13-Steering_Engine
            14-Encoder
            15-Pid
            16-Infrared_Sensor
            17-ESPNOW
            18-Odometry)

set(requires
            driver
            esp_driver_gpio
            esp_driver_spi
            esp_driver_gptimer
            esp_timer
            esp_driver_ledc
            esp_driver_mcpwm
            esp_adc
            esp_driver_uart
            esp_driver_i2c
            esp_driver_pcnt
            esp_wifi
            nvs_flash)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

target_compile_options(${COMPONENT_LIB} PRIVATE -ffast-math -O3 -Wno-error=format -Wno-format)
