idf_component_register(SRCS "reflectance.c"
                       INCLUDE_DIRS "include"
                       REQUIRES task-utils app-manager driver)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)