idf_component_register(SRCS "data_receive.c" "algo.c"
                       INCLUDE_DIRS "include"
                       REQUIRES task-utils mpu9255 hc-sr04 app-manager)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)