idf_component_register(SRCS "final_heading.c" "lp_fir_filter.c" "lp_iir_filter.c" "hp_iir_filter.c" "desired_heading.c" "heading_odometry.c" "photo_encoder.c" "position.c" "obstacle_avoidance.c" "hall.c" "data_receive.c" "heading_imu.c" "algo.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "priv-include"
                       REQUIRES driver task-utils mpu9255 hc-sr04 app-manager motor)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)