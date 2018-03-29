/************************************************************************************
 * configs/stm32f103-minimum/src/stm32_lcdbp.c
 *
 *   Copyright (C) 2018 Kyle Lei. All rights reserved.
 *   Author: Kyle Lei <leizhao2@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/pcf8574_lcd_backpack.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm32f103_minimum.h"

#if defined(CONFIG_I2C) && defined(CONFIG_LCD_BACKPACK)
/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_lcdbpinitialize
 *
 * Description:
 *   Initialize and register the LCD1602 with a PCF8574 backpack.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/slcd0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/
int stm32_lcdbpinitialize(FAR const char *devpath) {
	FAR struct i2c_master_s *i2c;
	int ret;

	/* Initialize I2C */

	i2c = stm32_i2cbus_initialize(1);

	if (!i2c) {
		syslog(LOG_ERR, "ERROR: Error initializing I2C bus");
		return -ENODEV;
	}

	struct pcf8574_lcd_backpack_config_s cfg = LCD_I2C_BACKPACK_CFG_ROBOT;

	cfg.addr = 0x3F;
	cfg.rows = 2;
	cfg.cols = 16;

	/* Then register the device */

	ret = pcf8574_lcd_backpack_register(devpath, i2c, &cfg);
	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Error registering LCD1602: %d\n", ret);
		return -ENODEV;
	}

	return ret;
}
#endif
