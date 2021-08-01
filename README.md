# OpenLidar
Open Lidar

The idea of this project is to use cheap laser distance modules to create 3D scans of the room. This can be done by rotating the laser distance module on 2 axis with the sensor centered on the junction of those two axis (aka: the sensor is on the center of rotation so it's always at the origin). The main goals of this project are to improve the means that cavers use to survey and map caves as citizen scientists, or to create point-clouds for your own purpose.

The project is developed in 3 parts. 1) The hardware for the gimbal. 2) The software to operate the gimbal. 3) Bluetooth/Serial interface for controlling the device with inputs. 

This is a work in progress and has been submitted to the Hackaday Prize. https://hackaday.io/project/11598-open-lidar

The original project had both hardware and software issues and was scrapped for the newer version located here. For older information you will need to checkout previous commits of this git repository. Issues included: not enough ram on the arduino mini pro to fill an SD card write block (extra SD writes = slow), hardware design of the body created too much friction for fast motor operation, hardware was unstable/wobbly, power supply was exceedingly large to fit in case, hardware was not enclosed in protective enclosure, the yaw motor could turn freely (and be knocked out of position easily), the SF-30B module had excessive noise per reading, etc.

The change in case hardware, microchip used, lidar sensor, and yaw motor assembly all necessitated a full rework of the project here.

The current state of the hardware:
- Yaw axis has been completely redesigned to use 30mm shaft through bearing for greater stability
- Pitch axis has been completely redesigned to use "fieldgoal" design to reduce friction and complexity of shaft
- Case parts have been modeled in Fusion 360 but need to finish printing/testing
- Re-selected new hardware to make use of S42B stepper drivers for simplicity and servo-like capabilities

The current status of the software is that it is capable of:
- Capturing data from LidarLitev3 using enhanced library for speeds of up to 2000/s (with ideal I2C conditions)
- Controlling 2 stepper motors 
- Receiving commands and outputing results over serial
- Write to SD card

Future items needed
- Scan Mode to prevent scanRoom from over-sampling near polar regions
- Add ability to read/write settings to ROM
- Add ability to change default settings
- Create progress indicators and time estimates
- Add ability to save as E57 point cloud file
- Add ability to save as xyz point cloud file
- Add routine for identifying 0 degrees
- Add ability to create custom filename (i.e. date and time)
