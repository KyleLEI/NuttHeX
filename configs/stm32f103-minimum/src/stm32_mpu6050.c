/************************************************************************************
 * configs/stm32f103-minimum/src/stm32_mpu6050.c
 *
 *   Author: Kyle Lei <leizhao2@gmail.com>
 *   Reference: stm32_veml6070.c, stm32_lm75.c
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mpu6050.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm32f103_minimum.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MPU6050)
/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_mpu6050initialize
 *
 * Description:
 *   Initialize and register the InvenSense MPU-6050 6-axis accelerometer/gyroscope.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mpu6050"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int stm32_mpu6050initialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(2);

  if (!i2c)
    {
	  syslog(LOG_ERR, "ERROR: Error initializing I2C bus");
      return -ENODEV;
    }

  /* Then register the device */

  ret = mpu6050_register(devpath, i2c, 0xAA);//FIXME: wrong address
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Error registering MPU6050: %d\n", ret);
    }

  return ret;
}
#endif
