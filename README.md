# OpenLidar
Open Lidar

The idea of this project is to use cheap laser distance modules to create 3D scans of the room. This can be done by rotating the laser distance module on 2 axis with the sensor centered on the junction of those two axis (aka: the sensor is on the center of rotation so it's always at the origin). The main goals of this project are to improve the means that cavers use to survey and map caves as citizen scientists. 

The project is developed in 3 parts. 1) The hardware for the gimbal. 2) The software to operate the gimbal. 3) Bluetooth/Serial interface for controlling the device with inputs. 

This is a work in progress and has been submitted to the Hackaday Prize. https://hackaday.io/project/11598-open-lidar

The current status of the software is that it is capable of:
- Controlling 2 stepper motors
- Checking a switching hall effect sensor to detect pitch motor placement
- Writing to a microSD card
- Connecting to a computer via Bluetooth (and outputing serial data)
