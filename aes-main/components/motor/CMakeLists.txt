idf_component_register(SRCS "motor.c"
                       INCLUDE_DIRS "include"
                       REQUIRES task-utils app-manager)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)