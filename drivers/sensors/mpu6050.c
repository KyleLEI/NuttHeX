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

/**Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 */

#ifndef CONFIG_MPU6050_SMPLRT
#  define CONFIG_MPU6050_SMPLRT 0x07
#endif

/** MPUREG_ACCEL_CONFIG [4:3]
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 */

#ifndef CONFIG_MPU6050_ACCEL_RANGE
#  define CONFIG_MPU6050_ACCEL_RANGE 0
#endif

/**
 * MPUREG_GYRO_CONFIG [4:3]
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 */

#ifndef CONFIG_MPU6050_GYRO_RANGE
#  define CONFIG_MPU6050_GYRO_RANGE 0
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

static int     mpu6050_i2c_read(FAR struct mpu6050_dev_s *priv,
        uint8_t const regaddr, FAR uint8_t *regval, int len);
static int 	   mpu6050_i2c_write(FAR struct mpu6050_dev_s *priv,
		uint8_t* regval,int len);
static int     mpu6050_write8(FAR struct mpu6050_dev_s *priv,
		uint8_t const regaddr, uint8_t regval);

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
 * Name: mpu6050_i2c_read
 *
 * Description:
 *   Read an arbitrary number of bytes starting at regaddr
 *
 ****************************************************************************/

static int mpu6050_i2c_read(FAR struct mpu6050_dev_s *priv,
                           uint8_t const regaddr, FAR uint8_t *regval, int len)
{
  struct i2c_config_s config;
  int ret = -1;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MPU6050_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

//  /* Write the register address to read from */
//
//  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
//  if (ret < 0)
//    {
//	  syslog(LOG_ERR,"i2c_write to 0x%02X -> 0x%02X failed: %d\n", priv->addr,regaddr,ret);
//      return ret;
//    }
//
//  /* Read "len" bytes from regaddr */
//
//  ret = i2c_read(priv->i2c, &config, regval,len);
//  if (ret < 0)
//    {
//	  syslog(LOG_ERR,"i2c_read from 0x%02X failed: %d\n", regaddr,ret);
//      return ret;
//    }
  uint8_t mpu_addr;

  for(mpu_addr=0x00;mpu_addr<127;mpu_addr++){
	  config.address=mpu_addr;
	  ret=i2c_writeread(priv->i2c,&config,&regaddr,1,regval,len);
	  if(ret>=0) break;
  }

  syslog(LOG_INFO,"MPU-6050 found at: 0x%02X, returning %d\n", mpu_addr,ret);
  return OK;
}

/****************************************************************************
 * Name: mpu6050_i2c_write
 *
 * Description:
 *   Write address/data to mpu6050
 *
 ****************************************************************************/

static int mpu6050_i2c_write(FAR struct mpu6050_dev_s *priv, uint8_t* regval,int len)
{
  struct i2c_config_s config;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_MPU6050_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

//  /* Write 8 bits to device */
//
//  ret = i2c_write(priv->i2c, &config, regval, len);
//  if (ret < 0)
//    {
//      syslog(LOG_ERR,"ERROR: i2c_write to addr [0x%02X] failed: %d\n",config.address, ret);
//    }
  uint8_t mpu_addr;

    for(mpu_addr=0x00;mpu_addr<127;mpu_addr++){
  	  config.address=mpu_addr;
  	  ret=i2c_write(priv->i2c, &config, regval, len);
  	  if(ret>=0) break;
    }

    syslog(LOG_INFO,"MPU-6050 found at: 0x%02X, returning %d\n", mpu_addr,ret);
  return ret;
}

/****************************************************************************
 * Name: mpu6050_write8
 *
 * Description:
 *   Write an arbitrary number of bytes starting at regaddr.
 *
 ****************************************************************************/

static int mpu6050_write8(FAR struct mpu6050_dev_s *priv,
                             uint8_t const regaddr, uint8_t regval)
{
  int ret;
  uint8_t data[2];

  /* Create the addr:val data */

  data[0] = regaddr;
  data[1] = regval;

  ret = mpu6050_i2c_write(priv, data, 2);

  if (ret < 0)
      {
        syslog(LOG_ERR,"ERROR: i2c_write8 failed: %d\n", ret);
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
 * 	This routine reads the contents of ACCEL & GYRO output registers, in the
 * 	order of MPUREG_ACCEL_XOUT_H	and the following 5 regs, and MPUREG_GYRO_XOUT_H
 * 	and the following 5 regs
 *
 * 	Further processing is required to combine H & L and converting according to
 * 	range settings to practical data in m/(s*s)
 *
 * 	One way of combining H & L is
 *
 * 	buffer[2 * i] << 8 + buffer[2 * i + 1]
 *
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

  if (buflen < 12)
    {
      snerr("ERROR: You need to read 6 signed 16-bit integer from this sensor!\n");
      return -EINVAL;
    }

  /* Read ACCEL registers consecutively*/

  ret = mpu6050_i2c_read(priv, MPUREG_ACCEL_XOUT_H, (FAR uint8_t *)&buffer[0], 6);
  if (ret < 0)
  {
	  snerr("ERROR: Error reading MPU6050!\n");
	  return ret;

  }

  /* Read ACCEL registers consecutively*/

  ret = mpu6050_i2c_read(priv, MPUREG_GYRO_XOUT_H, (FAR uint8_t *)&buffer[6], 6);
  if (ret < 0)
  {
	  snerr("ERROR: Error reading MPU6050!\n");
 	  return ret;
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
      syslog(LOG_ERR, "ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  //TODO: remove when done
  uint8_t temp=0;
  ret=mpu6050_i2c_read(priv,MPUREG_WHOAMI,&temp,1);
  if (ret < 0)
    {
  	  syslog(LOG_ERR, "ERROR: Failed to read WHOAMI: %d\n",ret);
  	  return ret;
    }
  	  syslog(LOG_INFO, "INFO: WHOAMI: [0x%02X]\n",temp);
//  	  return ret;

  /* Activate the device and take it out of sleep mode */

  ret = mpu6050_write8(priv, MPUREG_PWR_MGMT_1, 0x00);
  if (ret < 0)
  {
	  syslog(LOG_ERR, "ERROR: Failed to write MPUREG_PWR_MGMT_1: %d\n",ret);
	  return ret;
  }

  /* Set sampling rate of the device */

  ret = mpu6050_write8(priv, MPUREG_SMPLRT_DIV, CONFIG_MPU6050_SMPLRT);
  if (ret < 0)
  {
	syslog(LOG_ERR, "ERROR: Failed to write MPUREG_SMPLRT_DIV!\n");
	return ret;
  }

  /* Set accelerometer range */

  ret = mpu6050_write8(priv, MPUREG_ACCEL_CONFIG, CONFIG_MPU6050_ACCEL_RANGE<<3);
  if (ret < 0)
  {
	  syslog(LOG_ERR,"ERROR: Failed to write MPUREG_SMPLRT_DIV!\n");
  	return ret;
  }

   /* Set gyroscope range */

  ret = mpu6050_write8(priv, MPUREG_GYRO_CONFIG, CONFIG_MPU6050_ACCEL_RANGE<<3);
  if (ret < 0)
  {
	  syslog(LOG_ERR,"ERROR: Failed to write MPUREG_SMPLRT_DIV!\n");
    	return ret;
  }

  /* Register the character driver */

  ret = register_driver(devpath, &g_mpu6050_fops, 0666, priv);
  if (ret < 0)
    {
	  syslog(LOG_ERR,"ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSOR_MPU6050 */
