# Koruza-Accelerometer-module [![Build Status](https://travis-ci.org/IRNAS/koruza-accelerometer-module.svg?branch=master)](https://travis-ci.org/IRNAS/koruza-accelerometer-module)
The koruza-accelerometer-module is a small addon board/module for the Koruza unit. This module connects to the USB port of the koruza CM board and its main purpose is to collect and process the vibration data of the Koruza unit.

### Requirments
Requirements for the module are:
* small hardware form factor
* optional firmware upgrade inside the koruza unit
* USB connection to provide power and data
* collect and process vibration data, frequencies of interest: **from about 0.01 to 100 Hz**
* send processed data periodically, every 0.5 to 1 sec


### Hardware
There are several ways to measure mechanical vibrations and we will measure them using an MEMS-based accelerometer - [MPU-6050 accelerometer](link1_accelerometer). The MPU-6050 accelerometer is a triaxial MEMS type device, that can be programmed to measure from 2 _g_ to 16 _g_. The MPU-6050 sensor is mounted on the board [Flip32 All In One (Pro) Flight Controller V1.03](link2_module) which also has an:
* STM32F103 MCU,
* MS5611 Barometer,
* Onboard USB (USB to Serial CP210x)
* and as mentioned MPU6050.

Board dimensions are: 35 x 35 mm (30 x 30mm mounting holes), weight is 17g including wiring.

![hw_module][link3_hw_image]

### Software
Originally module should have been just the I/O interface between the accelerometer and the Koruza unit, but because the data of interest is form 0.01 to 100 Hz which meant that sampling frequency needs to be at least two times bigger than the maximum frequency, that was not possible. So the sampling frequency is least 200 Hz and this means that time between each data package which is sent form the accelerometer module to the koruza module is 5 ms. This could represent the problem for the Koruza CM because it is already running the koruza control software and has intensive communication with the move driver, SFP modules, etc. 

This means the MCU need to process the data before sending it. Data will be sent for all three axes x, y, and z. To be able the send all the important information the data will be divided into the several segments and send the vibration data for each axes in pairs of - peak value and average intensity. 

To collect information about peak value and average intensity over some period of time and for each axis we need to change from the time domain to frequency domain. This will be done using Furie transformation (FFT).

### Data
Like mentioned data is sent for every axis:
* x axis
  * average value for each range
    * avg_x[0], A - B Hz
    * avg_x[1], B - C Hz
    * avg_x[2], C - D Hz
    * avg_x[3], D - E Hz
  * maximum value for each range
    * max_x[0], A - B Hz
    * max_x[1], B - C Hz
    * max_x[2], C - D Hz
    * max_x[3], D - E Hz
* y axis
  * average value for each range
    * avg_y[0], A - B Hz
    * avg_y[1], B - C Hz
    * avg_y[2], C - D Hz
    * avg_y[3], D - E Hz
  * maximum value for each range
    * max_y[0], A - B Hz
    * max_y[1], B - C Hz
    * max_y[2], C - D Hz
    * max_y[3], D - E Hz
* z axis
  * average value for each range
    * avg_z[0], A - B Hz
    * avg_z[1], B - C Hz
    * avg_z[2], C - D Hz
    * avg_z[3], D - E Hz
  * maximum value for each range
    * max_z[0], A - B Hz
    * max_z[1], B - C Hz
    * max_z[2], C - D Hz
    * max_z[3], D - E Hz


This data are processed every 4 seconds and send to the Koruza compute module unit. 
Average values for all axis are from 0 to ~1000 when the module is not moving. 

---

## Licensing


#### License

Firmware and software originating from KORUZA Pro project, including Koruza Accelerometer Module, is licensed under [GNU GENERAL PUBLIC LICENSE v3](https://www.gnu.org/licenses/gpl-3.0.en.html).

What this means is that you can use this firmware and software without paying a royalty and knowing that you'll be able to use your version forever. You are also free to make changes but if you share these changes then you have to do so on the same conditions that you enjoy.

KORUZA, KORUZA Pro and IRNAS are all names and marks of Institute IRNAS Rače. You may use these names and terms only to attribute the appropriate entity as required by the Open Licence referred to above. You may not use them in any other way and in particular you may not use them to imply endorsement or authorization of any hardware that you design, make or sell.

[link1_accelerometer]: <datasheet_link>
[link2_module]: <https://hobbyking.com/en_us/flip32-naze32-all-in-one-pro.html>
[link3_hw_image]: <https://github.com/IRNAS/koruza-accelerometer-module/blob/master/Pics/hardware_module.png.png>
[link4_segment_module]: <https://github.com/IRNAS/koruza-accelerometer-module/blob/master/Pics/segment_diagram.png>



