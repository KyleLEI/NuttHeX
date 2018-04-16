/****************************************************************************
 * drivers/lcd/ssd1306_seg.c
 * Alphanumeric/Segment version of the SSD1306 driver
 *
 *   Copyright (C) 2018 Kyle Lei. All rights reserved.
 *   Author: Kyle Lei <leizhao2@gmail.com>
 *
 * This driver is based on the SSD1306 driver developed by Aaron Lee
 * <hello14blog@gmail.com> from www.heltec.cn,
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/lcd/ssd1306_seg.h>

#include "ssd1306_seg.h"

#if defined(CONFIG_LCD_SSD1306_SEG)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_SSD1306_SEG_I2CADDR
#  define CONFIG_SSD1306_SEG_I2CADDR 60
#endif

#ifndef CONFIG_SSD1306_SEG_I2CFREQ
#  define CONFIG_SSD1306_SEG_I2CFREQ 400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ssd1306_seg_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int ssd1306_seg_open(FAR struct file *filep);
static int ssd1306_seg_close(FAR struct file *filep);
static ssize_t ssd1306_seg_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t ssd1306_seg_write(FAR struct file *filep,
								 FAR const char *buffer, size_t buflen);
static int ssd1306_seg_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ssd1306_seg_fops =
{
  ssd1306_seg_open,            /* open */
  ssd1306_seg_close,           /* close */
  ssd1306_seg_read,            /* read */
  ssd1306_seg_write,           /* write */
  NULL,            			   /* seek */
  ssd1306_seg_ioctl           /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  ,NULL             /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  ,NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssd1306_writedat
 *
 * Description:
 *   Write an 8-bit value into SSD1306 at address 0x40
 *
 ****************************************************************************/

int ssd1306_writedat(FAR struct ssd1306_seg_dev_s *priv,uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - SSD1306_Reg_Address - SSD1306_Write_Data - STOP
   */

  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

  /* Setup to the data to be transferred.  Two bytes:  The SSD1306 register
   * address followed by one byte of data.
   */

  txbuffer[0]   = 0x40;
  txbuffer[1]   = regval;

  /* Setup 8-bit SSD1306 address write message */

  msg.frequency = CONFIG_SSD1306_SEG_I2CFREQ;  /* I2C frequency */
  msg.addr      = priv->addr;              /* 7-bit address */
  msg.flags     = 0;                       /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                /* Transfer from this address */
  msg.length    = 2;                       /* Send two bytes following the address
                                            * then STOP */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
  return ret;
}

/****************************************************************************
 * Name: ssd1306_writecmd
 *
 * Description:
 *   Write an 8-bit value into SSD1306 at address 0x00
 *
 ****************************************************************************/

int ssd1306_writecmd(FAR struct ssd1306_seg_dev_s *priv,uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - SSD1306_Reg_Address - SSD1306_Write_Data - STOP
   */

  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

  /* Setup to the data to be transferred.  Two bytes:  The SSD1306 register
   * address followed by one byte of data.
   */

  txbuffer[0]   = 0x00;
  txbuffer[1]   = regval;

  /* Setup 8-bit SSD1306 address write message */

  msg.frequency = CONFIG_SSD1306_SEG_I2CFREQ;  /* I2C frequency */
  msg.addr      = priv->addr;              /* 7-bit address */
  msg.flags     = 0;                       /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                /* Transfer from this address */
  msg.length    = 2;                       /* Send two bytes following the address
                                            * then STOP */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
  return ret;
}

void ssd1306_seg_init(FAR struct ssd1306_seg_dev_s *priv){
	nxsig_usleep(100000); //delay 100ms

	ssd1306_writecmd(priv, 0xAE); //display off
	ssd1306_writecmd(priv, 0x20);	//Set Memory Addressing Mode
	ssd1306_writecmd(priv, 0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_writecmd(priv, 0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_writecmd(priv, 0xc8);	//Set COM Output Scan Direction
	ssd1306_writecmd(priv, 0x00); //---set low column address
	ssd1306_writecmd(priv, 0x10); //---set high column address
	ssd1306_writecmd(priv, 0x40); //--set start line address
	ssd1306_writecmd(priv, 0x81); //--set contrast control register
	ssd1306_writecmd(priv, 0xff); // 0x00~0xff
	ssd1306_writecmd(priv, 0xa1); //--set segment re-map 0 to 127
	ssd1306_writecmd(priv, 0xa6); //--set normal display
	ssd1306_writecmd(priv, 0xa8); //--set multiplex ratio(1 to 64)
	ssd1306_writecmd(priv, 0x3F); //
	ssd1306_writecmd(priv, 0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_writecmd(priv, 0xd3); //-set display offset
	ssd1306_writecmd(priv, 0x00); //-not offset
	ssd1306_writecmd(priv, 0xd5); //--set display clock divide ratio/oscillator frequency
	ssd1306_writecmd(priv, 0xf0); //--set divide ratio
	ssd1306_writecmd(priv, 0xd9); //--set pre-charge period
	ssd1306_writecmd(priv, 0x22); //
	ssd1306_writecmd(priv, 0xda); //--set com pins hardware configuration
	ssd1306_writecmd(priv, 0x12);
	ssd1306_writecmd(priv, 0xdb); //--set vcomh
	ssd1306_writecmd(priv, 0x20); //0x20,0.77xVcc
	ssd1306_writecmd(priv, 0x8d); //--set DC-DC enable
	ssd1306_writecmd(priv, 0x14); //
	ssd1306_writecmd(priv, 0xaf); //--turn on oled panel

}

/****************************************************************************
 * Name: ssd1306_seg_setpos
 *
 * Description:
 *   This function sets the cursor at (x,y)
 *
 ****************************************************************************/
void ssd1306_seg_setpos(FAR struct ssd1306_seg_dev_s *priv,
		unsigned char x, unsigned char y)
{
	ssd1306_writecmd(priv, 0xb0+y);
	ssd1306_writecmd(priv, ((x&0xf0)>>4)|0x10);
	ssd1306_writecmd(priv, (x&0x0f)|0x01);
}

/****************************************************************************
 * Name: ssd1306_seg_fillregion
 *
 * Description:
 *   This function fills the region from line y0 starting at x0, to line y1
 *   ending at x1 with fill_data
 *
 ****************************************************************************/
void ssd1306_seg_fillregion(FAR struct ssd1306_seg_dev_s *priv,
		unsigned char x0, unsigned char y0, unsigned char x1,unsigned char y1,
		unsigned char fill_Data)
{
	uint8_t m, n;
	for (m = y0; m <= y1; m++)
	{
		ssd1306_writecmd(priv, 0xb0 + m);		//page0-page1
		ssd1306_writecmd(priv, 0x00);		//low column start address
		ssd1306_writecmd(priv, 0x10);		//high column start address
		for (n = x0; n <= x1; n++)
		{
			ssd1306_writedat(priv, fill_Data);
		}
	}
}

/****************************************************************************
 * Name: ssd1306_seg_fill
 *
 * Description:
 *   This function fills the entire screen with fill_data
 *
 ****************************************************************************/
void ssd1306_seg_fill(FAR struct ssd1306_seg_dev_s *priv,unsigned char fill_Data)
{
	uint8_t m, n;
	for (m = 0; m <= 8; m++)
	{
		ssd1306_writecmd(priv, 0xb0 + m);		//page0-page1
		ssd1306_writecmd(priv, 0x00);		//low column start address
		ssd1306_writecmd(priv, 0x10);		//high column start address
		for (n = 0; n <= 128; n++)
		{
			ssd1306_writedat(priv, fill_Data);
		}
	}
}

/****************************************************************************
 * Name: ssd1306_seg_drawbmp
 *
 * Description:
 *   This function draws a bitmap in the region (x0,y0), (x1,y1)
 *
 ****************************************************************************/
void ssd1306_seg_drawbmp(FAR struct ssd1306_seg_dev_s *priv, uint8_t x0,
		uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[])
{
	unsigned int j = 0;
	uint8_t x, y;

	if (y1 % 8 == 0)
		y = y1 / 8;
	else
		y = y1 / 8 + 1;
	for (y = y0; y < y1; y++)
	{
		ssd1306_seg_setpos(priv,x0, y);
		for (x = x0; x < x1; x++)
		{
			ssd1306_writedat(priv,BMP[j++]);
		}
	}
}

void ssd1306_seg_on(FAR struct ssd1306_seg_dev_s *priv)
{
	ssd1306_writecmd(priv, 0X8D);
	ssd1306_writecmd(priv, 0X14);
	ssd1306_writecmd(priv, 0XAF);
}

void ssd1306_seg_off(FAR struct ssd1306_seg_dev_s *priv)
{
	ssd1306_writecmd(priv, 0X8D);
	ssd1306_writecmd(priv, 0X10);
	ssd1306_writecmd(priv, 0XAE);
}

/****************************************************************************
 * Name: ssd1306_seg_showstr
 *
 * Description:
 *   This function prints a string ch[] at (x,y) using pre-defined font F6x8
 *
 ****************************************************************************/
void ssd1306_seg_showstr(FAR struct ssd1306_seg_dev_s *priv,
		uint8_t x, uint8_t y, const char ch[]){
	uint8_t c=0,i=0,j=0;
	while(ch[j] != '\0'){
		c = ch[j]-32;
		if(x>126){
			x = 0;
			y++;
		}
		ssd1306_seg_setpos(priv,x,y);
		for(i=0;i<6;i++)
			ssd1306_writedat(priv,F6x8[c][i]);
		x += 6;
		j++;
	}
}
/****************************************************************************
 * Name: ssd1306_seg_open
 *
 * Description:
 *   This function is called whenever the ssd1306 device is opened.
 *
 ****************************************************************************/

static int ssd1306_seg_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ssd1306_seg_close
 *
 * Description:
 *   This routine is called when the ssd1306 device is closed.
 *
 ****************************************************************************/

static int ssd1306_seg_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ssd1306_seg_write
 *
 *  Description: write the string buffer[2, ...] at x = buffer[0]-1 , y = buffer[1]-1
 ****************************************************************************/

static ssize_t ssd1306_seg_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
	int ret;
	FAR struct inode *inode;
	FAR struct ssd1306_seg_dev_s *priv;

	DEBUGASSERT(filep);
	inode = filep->f_inode;

	DEBUGASSERT(inode && inode->i_private);
	priv = (FAR struct ssd1306_seg_dev_s *) inode->i_private;

	/* Check if the user is writing the right size */

	if (buflen < 3)
	{
		snerr ("ERROR: You need to write {x, y, str} to the driver!\n");
		return -EINVAL;
	}

	ssd1306_seg_showstr(priv,buffer[0]-1,buffer[1]-1,buffer+2);
	return buflen-2;
}

/****************************************************************************
 * Name: ssd1306_read
 ****************************************************************************/

ssize_t ssd1306_seg_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ssd1306_ioctl
 ****************************************************************************/

int ssd1306_seg_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
	FAR struct inode *inode;
	FAR struct ssd1306_seg_dev_s *dev;
	int ret = OK;

	DEBUGASSERT(filep);
	inode = filep->f_inode;

	DEBUGASSERT(inode && inode->i_private);
	dev = inode->i_private;

	switch (cmd)
	{
	case SLCDIOC_FILLLINE:
	{
		FAR struct slcd_fill_s *fill_info =
				(FAR struct slcd_fill_s *) ((uintptr_t) arg);
		ssd1306_seg_fillregion(dev, 0, fill_info->start_line, 125,
				fill_info->end_line, fill_info->color);
	}
		break;

	case SLCDIOC_DRAWBMP:
	{
		FAR struct slcd_bmp_s *bmp_info =
				(FAR struct slcd_bmp_s *) ((uintptr_t) arg);
		ssd1306_seg_drawbmp(dev, bmp_info->x0, bmp_info->y0, bmp_info->x1,
				bmp_info->y1, bmp_info->bmp);
	}
		break;

	case SLCDIOC_CLEAR:
	{
		ssd1306_seg_fill(dev,0x00);
	}
		break;

	default:
		lcderr ("ERROR: Unrecognized cmd: %d\n", cmd);
		return -ENOTTY;

	}
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ssd1306_seg_register
 *
 * Description:
 *   Register the SSD1306 segment version  character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/slcd0"
 *   i2c - An instance of the I2C interface to use to communicate with SSD1306
 *   addr - The I2C address of the APDS9960.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ssd1306_seg_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

   /* Initialize the SSD1306 segment device structure */

   FAR struct ssd1306_seg_dev_s *priv =
     (FAR struct ssd1306_seg_dev_s *)kmm_malloc(sizeof(struct ssd1306_seg_dev_s));
   if (priv == NULL)
   {
     lcderr("ERROR: Failed to allocate instance\n");
     return -ENOMEM;
   }

   priv->i2c  = i2c;
   priv->addr = addr;

   ssd1306_seg_init(priv);

   /* Register the character driver */

   ret = register_driver(devpath, &g_ssd1306_seg_fops, 0666, priv);
   if (ret < 0)
   {
   	 lcderr("ERROR: Failed to register driver: %d\n", ret);
     kmm_free(priv);
    }

   /* Clear the screen */
   //ssd1306_seg_fill(priv,0x00); //optional if a bmp is to be drawn next

   /* Write some test stuff on the screen */
   ssd1306_seg_drawbmp(priv,0,0,128,8,(unsigned char*)hkust_logo);
   ssd1306_seg_showstr(priv,0,0,"ELEC3300 Group 59");
   ssd1306_seg_showstr(priv,0,1,"Kyle & Kaho");

   return ret;
}
#endif
