# NuttHeX
Core NuttX 7.23, with some adjustments

## Software Requirements

 - gcc-arm-none-eabi

## Build

Download the apps as well, put these two repos under the same directory. For example, nutthex.
```
mkdir nutthex
cd nutthex
git clone https://github.com/KyleLEI/NuttHeX.git nuttx
git clone https://github.com/KyleLEI/AppsHeX.git apps 
```
To configure for your own board, browse through available configs located in nuttx/configs for \<board\>/\<config\>. Then run
```
cd nuttx/tools
./configure.sh <board>/<config>
cd ..
make
```
Continue by uploading the built nuttx to the MCU

## Configuration
### Pin config
\<board\>/src/\<board\>.h
### Features config: 
\<board\>/\<config\>/defconfig
