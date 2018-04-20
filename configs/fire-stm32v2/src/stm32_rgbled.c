/************************************************************************************
 * configs/fire-stm32v2/src/stm32_rgbled.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/drivers/pwm.h>
#include <nuttx/leds/rgbled.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "stm32_pwm.h"
#include "fire-stm32v2.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define HAVE_RGBLED 1

#ifdef CONFIG_ARCH_LEDS || CONFIG_USERLED
#  undef HAVE_RGBLED // since they share the same pins
#endif

#ifndef CONFIG_PWM
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_STM32_TIM3
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_STM32_TIM3_PARTIAL_REMAP
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_STM32_TIM3_PWM
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_PWM_MULTICHAN
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_STM32_TIM3_CH2OUT
#  undef HAVE_PWM
#endif

#ifndef CONFIG_STM32_TIM3_CH3OUT
#  undef HAVE_PWM
#endif

#ifndef CONFIG_STM32_TIM3_CH4OUT
#  undef HAVE_PWM
#endif

#ifdef HAVE_RGBLED

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_rgbled_setup
 *
 * Description:
 *   Initial for support of a connected RGB LED using PWM.
 *
 ************************************************************************************/

int stm32_rgbled_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s    *ledrgb;
  struct pwm_info_s info;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      ledrgb = stm32_pwminitialize(RGBLED_RGBPWMTIMER);
      if (!ledrgb)
        {
          lederr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      /* Define frequency and duty cycle */

      info.frequency = 100;
      info.channels[0].channel = RGBLED_RPWMCHANNEL;
      info.channels[0].duty = 0;
      info.channels[1].channel = RGBLED_GPWMCHANNEL;
      info.channels[1].duty = 0;
      info.channels[2].channel = RGBLED_BPWMCHANNEL;
      info.channels[2].duty = 0;


      /* Initialize LED RGB */

      ledrgb->ops->setup(ledrgb);
      ledrgb->ops->start(ledrgb, &info);

      /* Register the RGB LED diver at "/dev/rgbled0" */

      ret = rgbled_register("/dev/rgbled0", ledrgb);
      if (ret < 0)
        {
          lederr("ERROR: rgbled_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#else
#  error "HAVE_RGBLED is undefined"
#endif /* HAVE_RGBLED */
