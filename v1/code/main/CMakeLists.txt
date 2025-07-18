idf_component_register(SRCS "h_bridge.c" "tinyusb_device.c" "digital_mic.c" "play.c" "signal_fft.c" "note_decode.c"
                            "uart_ctrl.c" "step_motor.c" "servo_motor.c" "main.c" "stepper_motor_encoder.c"
                       PRIV_REQUIRES usb
                       REQUIRES driver nvs_flash
                       INCLUDE_DIRS ".")

# espressif__tinyusb should match the current tinyusb dependency name
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

target_include_directories(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb")
target_sources(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb/usb_descriptors.c")
