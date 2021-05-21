add_executable(doorbirdpico
        doorbirdpico.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(doorbirdpico pico_stdlib hardware_pwm)

# create map/bin/hex file etc.
pico_add_extra_outputs(doorbirdpico)

pico_enable_stdio_usb(doorbirdpico 1)
pico_enable_stdio_uart(doorbirdpico 0)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/doorbirdpico.uf2
    DESTINATION .
)
