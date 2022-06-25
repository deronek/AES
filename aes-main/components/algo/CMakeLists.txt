idf_component_register(SRCS "border_recoil.c" "goal_heading.c" "final_heading.c" "lp_fir_filter.c" "lp_iir_filter.c" "hp_iir_filter.c" "heading_odometry.c" "photo_encoder.c" "position.c" "obstacle_avoidance.c" "hall.c" "data_receive.c" "heading_imu.c" "algo.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "priv-include"
                       REQUIRES driver task-utils mpu9255 hc-sr04 app-manager motor reflectance calc-utils)
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)