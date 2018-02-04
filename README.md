# NuttHeX
Core NuttX 7.23, with some adjustments

### Software Requirements

 - gcc-arm-none-eabi

### Build
Download the apps as well, put these two repos under the same directory. Then
```
cd nuttx
make
```
Continue by uploading the built nuttx to the MCU

### Configuration
Pin config: \<board\>/src/\<board\>.h
Features config: \<board\>/\<config\>/defconfig
