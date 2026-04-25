if(CMAKE_HOST_WIN32)
    string(REPLACE "\\" "/" IDF_PATH_UNIX "$ENV{IDF_PATH}")
else()
    set(IDF_PATH_UNIX "$ENV{IDF_PATH}")
endif()

idf_component_register(SRCS "terminal.c" "rgb_led.c" "key.c" "h_bridge.c" "tinyusb_device.c" "digital_mic.c" "play.c"
                            "signal_fft.c" "note_decode.c" "uart_ctrl.c" "step_motor.c" "ruler.c" "servo_motor.c"
                            "main.c" "stepper_motor_encoder.c" "nls.c"
                       PRIV_REQUIRES usb esp_app_format
                       REQUIRES driver nvs_flash
                       PRIV_INCLUDE_DIRS "${IDF_PATH_UNIX}/components/esp_driver_i2s"
                       INCLUDE_DIRS ".")

# espressif__tinyusb should match the current tinyusb dependency name
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

target_include_directories(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb")
target_sources(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb/usb_descriptors.c")
