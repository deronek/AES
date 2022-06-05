idf_component_register(SRCS "inv_mpu.c" "inv_mpu_dmp_motion_driver.c" "esp_port.c"
                       INCLUDE_DIRS "include"
                       REQUIRES i2c driver)

if(CONFIG_INV_GYRO_MPU6050)
    add_compile_definitions(MPU6050)
elseif(CONFIG_INV_GYRO_9150)
    add_compile_definitions(MPU9150)
elseif(CONFIG_INV_GYRO_MPU6500)
    add_compile_definitions(MPU6500)
elseif(CONFIG_INV_GYRO_MPU9250)
    add_compile_definitions(MPU9250)
endif()