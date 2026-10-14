idf_component_register(
    SRCS "src/telemetry_bus.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos log
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra -Werror -fno-exceptions -fno-rtti
)
