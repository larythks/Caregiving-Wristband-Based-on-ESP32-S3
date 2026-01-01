# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Add extra component directories
set(EXTRA_COMPONENT_DIRS
    "drivers/i2c"
    "drivers/onewire"
    "drivers/ds18b20"
    "drivers/max30102"
    "drivers/mpu6050"
    "drivers/oled"
    "drivers/button"
    "drivers/alarm_io"
    "drivers/battery"
    "drivers/usart"
    "drivers/i2s_mic"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_S3_Wristband)
