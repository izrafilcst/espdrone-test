# Fase 0.4 ativa: diag_camera_frame.cpp.
# Para trocar de firmware Fase 0, ajustar SRCS abaixo (ver guia_fase0_testes.md).
# Outros .cpp no diretorio permanecem fora de SRCS = nao compilam.
# nvs_flash + esp_event ficam sempre em REQUIRES (necessarios pelo main.cpp de producao).
idf_component_register(
    SRCS "diag_camera_frame.cpp"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_system esp_timer log nvs_flash esp_event espressif__esp32-camera
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Werror
    -fno-exceptions
    -fno-rtti
)
