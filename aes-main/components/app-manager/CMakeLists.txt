idf_component_register(SRCS "app_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES task-utils mpu9255 i2c algo hc-sr04 ble)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)