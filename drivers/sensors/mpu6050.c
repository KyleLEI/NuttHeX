/****************************************************************************
 * drivers/sensors/mpu6050.c
 *
 *   Author: Kyle Lei <leizhao2@gmail.com>
 *   Reference: http://bbs.elecfans.com/jishu_485014_1_1.html
 *   		    https://github.com/Harinadha/STM32_MPU6050lib
 *   		    veml6070.c, lm75.c, apds9960.c
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mpu6050.h>

//TODO: remove these two lines to enable macro guard
#define CONFIG_SENSORS_MPU6050
#define CONFIG_I2C

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MPU6050)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_MPU6050_I2C_FREQUENCY
#  define CONFIG_MPU6050_I2C_FREQUENCY 100000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpu6050_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     mpu6050_read8(FAR struct mpu6050_dev_s *priv, int offset,
                 FAR uint8_t *regval);
static int     mpu6050_write8(FAR struct mpu6050_dev_s *priv,
                 uint8_t regval);

/* Character driver methods */

static int     mpu6050_open(FAR struct file *filep);
static int     mpu6050_close(FAR struct file *filep);
static ssize_t mpu6050_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t mpu6050_write(FAR struct file *filep,
                 FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mpu6050_fops =
{
  mpu6050_open,   /* open */
  mpu6050_close,  /* close */
  mpu6050_read,   /* read */
  mpu6050_write,  /* write */
  NULL,            /* seek */
  NULL             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL           /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu6050_read8
 *
 * Description:
 *   Read 8-bit register
 *
 ****************************************************************************/

static int mpu6050_read8(FAR struct mpu6050_dev_s *priv, int offset,
                            FAR uint8_t *regval)
{
  struct i2c_config_s config;
  uint8_t data[1];
  int ret = -1;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MPU6050_I2C_FREQUENCY;
  config.address   = priv->addr + offset;
  config.addrlen   = 7;

  /* Read 8-bits from the device */

  ret = i2c_read(priv->i2c, &config, data, 1);
  if (ret < 0)
    {
      snerr ("i2c_read failed: %d\n", ret);
      return ret;
    }

  /* Copy the content of the buffer to the location of the uint8_t pointer */

  *regval = data[0];

  sninfo("value: %08x ret: %d\n", *regval, ret);
  return OK;
}

/****************************************************************************
 * Name: veml6070_write8
 *
 * Description:
 *   Write from an 8-bit register
 *
 ****************************************************************************/

static int mpu6050_write8(FAR struct mpu6050_dev_s *priv, uint8_t regval)
{
  struct i2c_config_s config;
  int ret;

  sninfo("value: %02x\n", regval);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MPU6050_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write 8 bits to device */

  ret = i2c_write(priv->i2c, &config, &regval, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mpu6050_open
 *
 * Description:
 *   This function is called whenever the MPU6050 device is opened.
 *
 ****************************************************************************/

static int mpu6050_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mpu6050_close
 *
 * Description:
 *   This routine is called when the MPU6050 device is closed.
 *
 ****************************************************************************/

static int mpu6050_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mpu6050_read
 ****************************************************************************/
//TODO: continue reading the right size and write to buffer
static ssize_t mpu6050_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  int ret;
  FAR struct inode         *inode;
  FAR struct mpu6050_dev_s *priv;
  int msb = 1;
  uint16_t regdata;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mpu6050_dev_s *)inode->i_private;

  /* Check if the user is reading the right size */

  if (buflen != 2)
    {
      snerr("ERROR: You need to read 2 bytes from this sensor!\n");
      return -EINVAL;
    }

  /* Enable the sensor */

  ret = veml6070_write8(priv, VEML6070_CMD_RSV & ~VEML6070_CMD_SD);
  if (ret < 0)
    {
      snerr("ERROR: Failed to enable the VEML6070!\n");
      return -EINVAL;
    }

  /* 1T for Rset 270Kohms is 125ms */

  nxsig_usleep(125000);

  /* Read the MSB first */

  ret = veml6070_read8(priv, msb, (FAR uint8_t *) &regdata);
  if (ret < 0)
    {
      snerr("ERROR: Error reading light sensor!\n");
      return ret;
    }

  buffer[1] = regdata;

  /* Read the LSB */

  msb = 0;
  ret = veml6070_read8(priv, msb, (FAR uint8_t *) &regdata);
  if (ret < 0)
    {
      snerr("ERROR: Error reading light sensor!\n");
      return ret;
    }

  buffer[0] = regdata;

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((buffer[1] << 16) ^ buffer[0]);

  return buflen;
}

/****************************************************************************
 * Name: mpu6050_write
 ****************************************************************************/

static ssize_t mpu6050_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

#endif /* CONFIG_I2C && CONFIG_SENSOR_MPU6050 */
