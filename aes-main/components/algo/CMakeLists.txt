idf_component_register(SRCS "desired_heading.c" "heading_odometry.c" "comp_iir_filter.c" "obstacle_avoidance.c" "position.c" "photo_encoder.c" "hall.c" "data_receive.c" "heading_imu.c" "algo.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver task-utils mpu9255 hc-sr04 app-manager motor)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)