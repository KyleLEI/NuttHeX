/****************************************************************************
 * include/nuttx/sensors/lm75.h
 *
 *	 Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *	 Copyright (c) 2017 Kyle Lei. All rights reserved.
 *   Author: Kyle Lei <leizhao2@gmail.com>
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_MPU6050_H
#define __INCLUDE_NUTTX_SENSORS_MPU6050_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MPU6050)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_MPU6050_I2C - Enables support for the MPU6050 driver
 */

/* Device I2C Address */

/* MPU 6000 registers */

#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_TRIM1			0x0D
#define MPUREG_TRIM2			0x0E
#define MPUREG_TRIM3			0x0F
#define MPUREG_TRIM4			0x10
#define MPU_GYRO_DLPF_CFG_256HZ_NOLPF2	0x00
#define MPU_GYRO_DLPF_CFG_188HZ		0x01
#define MPU_GYRO_DLPF_CFG_98HZ		0x02
#define MPU_GYRO_DLPF_CFG_42HZ		0x03
#define MPU_GYRO_DLPF_CFG_20HZ		0x04
#define MPU_GYRO_DLPF_CFG_10HZ		0x05
#define MPU_GYRO_DLPF_CFG_5HZ		0x06
#define MPU_GYRO_DLPF_CFG_2100HZ_NOLPF	0x07
#define MPU_DLPF_CFG_MASK		0x07

/* ICM2608 specific registers */

#define ICMREG_ACCEL_CONFIG2		0x1D
#define ICM_ACC_DLPF_CFG_1046HZ_NOLPF	0x00
#define ICM_ACC_DLPF_CFG_218HZ		0x01
#define ICM_ACC_DLPF_CFG_99HZ		0x02
#define ICM_ACC_DLPF_CFG_44HZ		0x03
#define ICM_ACC_DLPF_CFG_21HZ		0x04
#define ICM_ACC_DLPF_CFG_10HZ		0x05
#define ICM_ACC_DLPF_CFG_5HZ		0x06
#define ICM_ACC_DLPF_CFG_420HZ		0x07

/* Utilities */

#define MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30
#define MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define MPU6050_DEFAULT_ONCHIP_FILTER_FREQ			42

#define MPU6050_ONE_G					9.80665f


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mpu6050_register
 *
 * Description:
 *   Register the MPU6050 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mpu6050"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *              MPU6050
 *   addr    - The I2C address of the MPU6050.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int mpu6050_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_MPU6050_I2C */
#endif /* __INCLUDE_NUTTX_SENSORS_MPU6050_H */
