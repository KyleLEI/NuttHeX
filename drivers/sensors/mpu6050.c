/****************************************************************************
 * drivers/sensors/mpu6050.c
 *
 *   Author: Kyle Lei <leizhao2@gmail.com>
 *   Reference: http://bbs.elecfans.com/jishu_485014_1_1.html
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lm75.h>
#include <nuttx/random.h>
