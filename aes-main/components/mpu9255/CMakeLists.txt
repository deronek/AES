idf_component_register(SRCS "mpu9255.c"
                       INCLUDE_DIRS "include"
                       REQUIRES motion-driver task-utils app-manager)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)