# NuttHeX
Core NuttX 7.23, with some adjustments

## Software Requirements

 - [gcc-arm-none-eabi](https://launchpad.net/gcc-arm-embedded/+download)
 - [kconfig-frontends](http://ymorin.is-a-geek.org/projects/kconfig-frontends)

If you plan to build the software on macOS, the required software can be installed through [Homebrew](https://brew.sh) easily, thanks to PX4. After the installation of Homebrew, run
```
brew tap PX4/px4
brew install gcc-arm-none-eabi
brew install kconfig-frontends
```
## Build

Download the apps as well, put these two repos under the same directory. For example, nutthex.
```
mkdir nutthex
cd nutthex
git clone https://github.com/KyleLEI/NuttHeX.git nuttx
git clone https://github.com/KyleLEI/AppsHeX.git apps 
```
To configure for your own board, browse through available configs located in nuttx/configs for \<board\>/\<config\>. For example, fire-stm32v2/nsh, run
```
cd nuttx
./tools/configure.sh fire-stm32v2/nsh
make
```
Continue by uploading the built nuttx to the MCU, you may find [GNU MCU Eclipse plug-ins](https://gnu-mcu-eclipse.github.io/plugins/install/) useful.

If you are using JLinkExe
```
JLinkExe
JLink>loadbin nuttx.hex 0x0
Device>STM32F103VE
TIF>s
Speed> 
```

To connect to the shell of NuttX, run
```
[Linux] screen /dev/ttyUSB0 115200 8N1
[OSX] screen /dev/tty.wchusbserial1410 115200 8N1
```
To exit, use ^A k


## New Board Support
 - Robomasters Development Board (nsh, userled)
 - Fire MINI-V3 (nsh, userled)
## New Device Driver
 - MPU6050
 - LCD1602 for STM32F103-minimum
## Bug Fix
 - APDS9960 work queue bug
