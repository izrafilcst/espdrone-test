idf_component_register(
    SRCS "src/imu_mpu6050.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos log esp_driver_i2c esp_driver_gpio esp_task_wdt esp_timer main
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra -Werror -fno-exceptions -fno-rtti
)
