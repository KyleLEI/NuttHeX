/************************************************************************************
 * configs/stm32f103-minimum/src/stm32_ssd1306seg.c
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
#include <nuttx/lcd/ssd1306_seg.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm32f103_minimum.h"

#if defined(CONFIG_I2C) && defined(CONFIG_LCD_SSD1306_SEG)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define SSD1306_SEG_I2C_PORTNO 1   /* On I2C1 */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_ssd1306seginitialize
 *
 * Description:
 *   Initialize and register the alphanumerical version of the SSD1306 driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/slcd0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int stm32_ssd1306seginitialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(SSD1306_SEG_I2C_PORTNO);

  if (!i2c)
    {
	  syslog(LOG_ERR, "ERROR: Error initializing I2C bus");
      return -ENODEV;
    }

  /* Then register the device */

  ret = ssd1306_seg_register(devpath, i2c, 60);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Error registering SSD1306_SEG: %d\n", ret);
      return -ENODEV;
    }

  return ret;
}

#endif
