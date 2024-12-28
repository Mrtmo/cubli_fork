set(COMPONENT_SRCS "button.c" "btn_ctrl.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_PRIV_REQUIRES "driver" "esp_timer")
register_component()