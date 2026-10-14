idf_component_register(
    SRCS "src/motor_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos log esp_driver_mcpwm esp_task_wdt main
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra -Werror -fno-exceptions -fno-rtti
)
