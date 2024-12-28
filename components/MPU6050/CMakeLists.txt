set(COMPONENT_SRCS "mmpu6050.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_PRIV_REQUIRES "driver" "i2c_device" "esp_timer")
register_component()