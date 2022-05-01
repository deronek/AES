idf_component_register(SRCS "ble.c"
                    INCLUDE_DIRS "include"
                    REQUIRES hc-sr04 task-utils mpu9255 app-manager)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)