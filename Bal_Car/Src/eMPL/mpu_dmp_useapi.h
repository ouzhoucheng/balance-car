#ifndef __MPU_DMP_USEAPI_H
#define __MPU_DMP_USEAPI_H

#include "stm32f1xx_hal.h"

void gyro_data_ready_cb(void);
uint8_t mpu_dmp_init(void);
uint8_t dmp_getdata(void);

#endif  // __MPU_DMP_USEAPI_H
