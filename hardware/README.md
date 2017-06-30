# Koruza accelerometer hardware.
![hardware_pic](https://github.com/IRNAS/koruza-accelerometer-module/blob/master/Pics/flip32aio-1.02-3.jpg)

Hardware is based on the FLIP32 AIO board. Main characteristics are:
* STM103 MCU
* Only 35x35mm, mount holes 30x30mm
* Onboard USB
* For switch the usb connection, USB to Flip32 or USB to OSD or Flip32 to OSD
* MPU6050 Accelerometer
* MS5611 Barometer


#### Board configuration:
* LED1 - PB4 (GPIO)/ PIN 40
* LED2 - PB3 (GPIO)/ PIN 39
* USB TO UART
  * RX - USART1-RX / PIN 31
  * TX - USART1-TX / PIN 30
* Boot
  * BOOT1 - PULL-DOWN / GND
  * BOOT0 - PULL-DOWN : PULL-UP 
* MPU6050 I2C2
  * SCL2 PB10 / PIN21
  * SDA2 PB11 / PIN22
* MS5611 I2C2
  * SCL2 PB10 / PIN21
  * SDA2 PB11 / PIN22 

#### I2C addresses:
* MPU6050 0x68
* MS5611  0x77
