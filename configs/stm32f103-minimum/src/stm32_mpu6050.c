/************************************************************************************
 * configs/stm32f103-minimum/src/stm32_mpu6050.c
 *
 *   Author: Kyle Lei <leizhao2@gmail.com>
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mpu6050.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm32f103_minimum.h"

//TODO: remove comment
#define CONFIG_SENSORS_MPU6050
#define CONFIG_I2C
#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MPU6050)

#endif
