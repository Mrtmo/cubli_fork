set(COMPONENT_SRCS "motor2204.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_PRIV_REQUIRES "driver" "esp_timer" "AS5600" "i2c_device")
register_component()