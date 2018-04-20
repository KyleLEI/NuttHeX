/****************************************************************************
 * drivers/rgbled.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/leds/rgbled.h>

#include <arch/irq.h>

#ifdef CONFIG_RGBLED

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct rgbled_upperhalf_s
{
  uint8_t           crefs;    /* The number of times the device has been opened */
  volatile bool     started;  /* True: pulsed output is being generated */
  sem_t             exclsem;  /* Supports mutual exclusion */
#ifdef CONFIG_PWM_MULTICHAN

  struct pwm_info_s ledrgb;     /* Pulsed output for LED RGB*/
  struct pwm_lowerhalf_s *devledrgb;

#else
  struct pwm_info_s ledr;     /* Pulsed output for LED R*/
  struct pwm_info_s ledg;     /* Pulsed output for LED G*/
  struct pwm_info_s ledb;     /* Pulsed output for LED B*/
  struct pwm_lowerhalf_s *devledr;
  struct pwm_lowerhalf_s *devledg;
  struct pwm_lowerhalf_s *devledb;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rgbled_open(FAR struct file *filep);
static int     rgbled_close(FAR struct file *filep);
static ssize_t rgbled_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t rgbled_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rgbledops =
{
  rgbled_open,  /* open */
  rgbled_close, /* close */
  rgbled_read,  /* read */
  rgbled_write, /* write */
  0,            /* seek */
  0             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0           /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rgbled_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ****************************************************************************/

static int rgbled_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct rgbled_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  ledinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      lederr("ERROR: nxsem_wait failed: %d\n", ret);
      DEBUGASSERT(ret == -EINTR);
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  nxsem_post(&upper->exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: rgbled_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ****************************************************************************/

static int rgbled_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct rgbled_upperhalf_s *upper = inode->i_private;
  int                         ret;

  ledinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      lederr("ERROR: nxsem_wait failed: %d\n", ret);
      DEBUGASSERT(ret == -EINTR);
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }

  nxsem_post(&upper->exclsem);
  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: rgbled_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t rgbled_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: rgbled_lightness
 *
 * Description:
 *   Convert an 8-bit color level to a 16-bit PWM command, using a
 *   piecewise linear approximation of the CIE 1931 lightness formula.
 *
 ****************************************************************************/

#ifdef CONFIG_RGBLED_LIGHTNESS_CORRECTION
static unsigned short rgbled_lightness(unsigned char color_level)
{
  unsigned int lut_index;
  unsigned short pwm_cmd = 0;

  static const unsigned char lut_color_in[9] =
    {
      0x00, 0x20, 0x40, 0x60, 0x80, 0xa0, 0xc0, 0xe0, 0xff
    };

  static const unsigned short lut_pwm_out[9] =
    {
      0x0000, 0x03d1, 0x0b62, 0x1952, 0x2f93,
      0x5015, 0x7ccb, 0xb7a7, 0xffff
    };

  for (lut_index = 0; lut_index < sizeof(lut_color_in); ++lut_index)
    {
      if (lut_color_in[lut_index] >= color_level)
        {
          break;
        }
    }

  if (lut_index < sizeof(lut_color_in))
    {
      if (lut_color_in[lut_index] == color_level)
        {
          pwm_cmd = lut_pwm_out[lut_index];
        }
      else
        {
          pwm_cmd =  (unsigned short)(lut_pwm_out[lut_index - 1] +
            (int)(lut_pwm_out[lut_index] - lut_pwm_out[lut_index - 1]) *
            (int)(color_level - lut_color_in[lut_index - 1]) /
            (int)(lut_color_in[lut_index] - lut_color_in[lut_index - 1]));
        }
    }

  return pwm_cmd;
}
#endif /* CONFIG_RGBLED_LIGHTNESS_CORRECTION */

/****************************************************************************
 * Name: rgbled_write
 *
 * Description:
 *   A write method which parses an HTML-style RGB string like "#FF8833"
 *   into color values, and sends them to the lower-half PWM drivers.
 *
 ****************************************************************************/

static ssize_t rgbled_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{

  FAR struct inode *inode = filep->f_inode;
  FAR struct rgbled_upperhalf_s *upper = inode->i_private;

#ifdef CONFIG_PWM_MULTICHAN
  FAR struct pwm_lowerhalf_s *ledrgb = upper->devledrgb;
#else
  FAR struct pwm_lowerhalf_s *ledr = upper->devledr;
  FAR struct pwm_lowerhalf_s *ledg = upper->devledg;
  FAR struct pwm_lowerhalf_s *ledb = upper->devledb;
#endif

  unsigned int red;
  unsigned int green;
  unsigned int blue;
  char color[3];

  /* We need to receive a string #RRGGBB = 7 bytes */

  if (buffer == NULL || buflen < 7)
    {
      /* Well... nothing to do */

      return -EINVAL;
    }

  /* Check if it is a color format */

  if (buffer[0] != '#')
    {
      /* The color code needs to start with # */

      return -EINVAL;
    }

  /* Move buffer to next character */

  buffer++;

  color[0] = buffer[0];
  color[1] = buffer[1];
  color[2] = '\0';

  red = strtol(color, NULL, 16);

  color[0] = buffer[2];
  color[1] = buffer[3];
  color[2] = '\0';

  green = strtol(color, NULL, 16);

  color[0] = buffer[4];
  color[1] = buffer[5];
  color[2] = '\0';

  blue = strtol(color, NULL, 16);

  /* Sane check */

  if (red > 255)
    {
      red = 255;
    }

  if (green > 255)
    {
      green = 255;
    }

  if (blue > 255)
    {
      blue = 255;
    }

  /* Convert 8bit to 16bits */

#ifdef CONFIG_RGBLED_LIGHTNESS_CORRECTION

  red   = rgbled_lightness((unsigned char)red);
  green = rgbled_lightness((unsigned char)green);
  blue  = rgbled_lightness((unsigned char)blue);

#else

  red   = (red   << 8) | red;
  green = (green << 8) | green;
  blue  = (blue  << 8) | blue;

#endif

#ifdef CONFIG_RGBLED_INVERT
  red   ^= 0xffff;
  green ^= 0xffff;
  blue  ^= 0xffff;
#endif
#ifdef CONFIG_PWM_MULTICHAN

  upper->ledrgb.frequency = 100;
  upper->ledrgb.channels[0].duty = red;
  upper->ledrgb.channels[1].duty = green;
  upper->ledrgb.channels[2].duty = blue;

  ledrgb->ops->start(ledrgb, &upper->ledrgb);

#else
  /* Setup LED R */

  upper->ledr.frequency = 100;
  upper->ledr.duty = red;

  ledr->ops->start(ledr, &upper->ledr);

  /* Setup LED G */

  upper->ledg.frequency = 100;
  upper->ledg.duty = green;

  ledg->ops->start(ledg, &upper->ledg);

  /* Setup LED B */

  upper->ledb.frequency = 100;
  upper->ledb.duty = blue;

  ledb->ops->start(ledb, &upper->ledb);
#endif

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rgbled_register
 *
 * Description:
 *   This function binds three instances of a "lower half" PWM driver with
 *   the "upper half" RGB LED device and registers that device so that can
 *   be used by application code.
 *
 *
 * Input parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/rgdbled0", "/dev/rgbled1", etc.  where the driver path
 *     differs only in the "minor" number at the end of the device name.
 *   ledrgb (with multi-channel support) - A pointer to an instance of lower half PWM driver.
 *   ledr, ledg, and ledb (if not multi-channel) - A pointer to an instance of lower half PWM
 *     drivers for the red, green, and blue LEDs, respectively.  These
 *     instances will be bound to the RGB LED driver and must persists as
 *     long as that driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/
#ifdef CONFIG_PWM_MULTICHAN

int rgbled_register(FAR const char *path, FAR struct pwm_lowerhalf_s *ledrgb,
		FAR struct pwm_info_s *info)
{
  FAR struct rgbled_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct rgbled_upperhalf_s *)
    kmm_zalloc(sizeof(struct rgbled_upperhalf_s));

  if (!upper)
    {
      lederr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by
   * kmm_zalloc())
   */

  nxsem_init(&upper->exclsem, 0, 1);
  upper->devledrgb = ledrgb;

  /* Specify channels to use, this is different from the traditional RGBLED driver */
  upper->ledrgb.channels[0] = info->channels[0];
  upper->ledrgb.channels[1] = info->channels[1];
  upper->ledrgb.channels[2] = info->channels[2];

  /* Register the PWM device */

  ledinfo("Registering %s\n", path);
  return register_driver(path, &g_rgbledops, 0666, upper);
}

#else
int rgbled_register(FAR const char *path, FAR struct pwm_lowerhalf_s *ledr,
                                          FAR struct pwm_lowerhalf_s *ledg,
                                          FAR struct pwm_lowerhalf_s *ledb)
{
  FAR struct rgbled_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct rgbled_upperhalf_s *)
    kmm_zalloc(sizeof(struct rgbled_upperhalf_s));

  if (!upper)
    {
      lederr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by
   * kmm_zalloc())
   */

  nxsem_init(&upper->exclsem, 0, 1);
  upper->devledr = ledr;
  upper->devledg = ledg;
  upper->devledb = ledb;

  /* Register the PWM device */

  ledinfo("Registering %s\n", path);
  return register_driver(path, &g_rgbledops, 0666, upper);
}
#endif

#endif /* CONFIG_RGBLED */
