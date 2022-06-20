idf_component_register(SRCS "i2c.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES driver app-manager)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 4)