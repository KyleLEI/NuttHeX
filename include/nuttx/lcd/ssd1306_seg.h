/****************************************************************************
 * include/nuttx/lcd/ssd1306_seg.h
 *
 *   Copyright (C) 2018 Kyle Lei. All rights reserved.
 *   Author: leizhao2@gmail.com
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
 ****************************************************************************/
#ifndef __INCLUDE_NUTTX_LCD_SSD1306_SEG_H
#define __INCLUDE_NUTTX_LCD_SSD1306_SEG_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/i2c/i2c_master.h>
#include <sys/ioctl.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* IOCTL commands that is supported by the SSD1306_SEG driver */

/* SLCDIOC_FILLLINE:  Fill the specific lines with color
 *
 * argument:  Pointer to struct slcd_fill_s, lines specifies in which will be
 * 				filled
 */

#define SLCDIOC_FILLLINE  _SLCDIOC(0x0010)

/* SLCDIOC_DRAWBMP:  Draw a BMP at the specified position
 *
 * argument:  Pointer to struct slcd_bmp_s, bmp specified in which will be shown
 * 				in a rectagular region from (x0,y0) to (x1,y1)
 */

#define SLCDIOC_DRAWBMP  _SLCDIOC(0x0011)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Used with the SLCDIOC_FILLLINE ioctl call */
struct slcd_fill_s{
	uint8_t start_line;
	uint8_t end_line;
	uint8_t color;
};

/* Used with the SLCDIOC_SHOWBMP ioctl call */
struct slcd_bmp_s{
	uint8_t x0;
	uint8_t y0;
	uint8_t x1;
	uint8_t y1;
	uint8_t* bmp;
};

/**************************************************************************************
 * Public Data
 **************************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
int ssd1306_seg_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);
#ifdef __cplusplus
}
#endif


#endif /* __INCLUDE_NUTTX_LCD_SSD1306_SEG_H */
