# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(digital_ruler_instrument)

# force fresh build timestamp in esp_app_desc
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}.elf
    PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E touch "${IDF_PATH}/components/esp_app_format/esp_app_desc.c"
    COMMENT "Forcing fresh build timestamp in esp_app_desc"
)
