#ifndef IMU_DATA_STRUCT_IMU_H_
#define IMU_DATA_STRUCT_IMU_H_

typedef struct
{
    float pos[3];
} gps_data;

typedef struct
{
    float rpy[3];
    float quat[4];
    float acc[3];
    float gyro[3];
    float freeAcc[3];
} imu_data;

#endif
