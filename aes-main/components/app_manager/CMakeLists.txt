idf_component_register(SRCS "tasks.c" "app_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES task-utils mpu9255 i2c)