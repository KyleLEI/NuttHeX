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

static int     mpu6050_read8(FAR struct mpu6050_dev_s *priv,
        uint8_t const regaddr, FAR uint8_t *regval);
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
 *   Read an arbitrary number of bytes starting at regaddr
 *
 ****************************************************************************/

static int mpu6050_read8(FAR struct mpu6050_dev_s *priv,
                           uint8_t const regaddr, FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int ret = -1;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MPU6050_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address to read from */

  ret = i2c_write(priv->addr, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr ("i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read "len" bytes from regaddr */

  ret = i2c_read(priv->addr, &config, regval,1);
  if (ret < 0)
    {
      snerr ("i2c_read failed: %d\n", ret);
      return ret;
    }

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
 *
 * Description:
 * 	This routine reads the contents of registers indicated in regs[12]
 * 	to the provided buffer.
 *
 * 	Further processing is required to combine H & L and converting according to
 * 	range settings to practical data in m/(s*s)
 *
 ****************************************************************************/

static ssize_t mpu6050_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  int ret;
  FAR struct inode         *inode;
  FAR struct mpu6050_dev_s *priv;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mpu6050_dev_s *)inode->i_private;

  /* Check if the user is reading the right size */

  if (buflen != 12)
    {
      snerr("ERROR: You need to read 6 signed 16-bit integer from this sensor!\n");
      return -EINVAL;
    }

  static const uint8_t regaddrs[12]={
		  MPUREG_ACCEL_XOUT_H,
		  MPUREG_ACCEL_XOUT_L,
		  MPUREG_ACCEL_YOUT_H,
		  MPUREG_ACCEL_YOUT_L,
		  MPUREG_ACCEL_ZOUT_H,
		  MPUREG_ACCEL_ZOUT_L,

		  MPUREG_GYRO_XOUT_H,
		  MPUREG_GYRO_XOUT_L,
		  MPUREG_GYRO_YOUT_H,
		  MPUREG_GYRO_YOUT_L,
		  MPUREG_GYRO_ZOUT_H,
		  MPUREG_GYRO_ZOUT_L
  };

  int regnum;
  for (regnum = 0;regnum < 12;regnum++){
	  ret = mpu6050_read8(priv, regaddrs[regnum],&buffer[regnum]);
	    if (ret < 0)
	      {
	        snerr("ERROR: Error reading MPU6050!\n");
	        return ret;
	      }
  }

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu6050_register
 *
 * Description:
 *   Register the MPU6050 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mpu6050"
 *   i2c - An instance of the I2C interface to use to communicate with MPU6050
 *   addr - The I2C address of the MPU6050.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mpu6050_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the MPU6050 device structure */

  FAR struct mpu6050_dev_s *priv =
    (FAR struct mpu6050_dev_s *)kmm_malloc(sizeof(struct mpu6050_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the VEML6070!\n");
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_mpu6050_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSOR_MPU6050 */
