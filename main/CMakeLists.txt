# idf_component_register(SRCS "main.c" "Kalman_filter.c" "mpu6050.c"
#                     INCLUDE_DIRS ".")
set(SOURCES 
    main.c
    Kalman/kalman.c
    MPU6050/mpu6050.c
)

set(INCLUDES 
    .
    Kalman
    MPU6050
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS ${INCLUDES})
