# OpenLidar
Open Lidar

The idea of this project is to use cheap laser distance modules to create 3D scans of the room. This can be done by rotating the laser distance module on 2 axis with the sensor centered on the junction of those two axis (aka: the sensor is on the center of rotation so it's always at the origin). The main goals of this project are to improve the means that cavers use to survey and map caves as citizen scientists. 

The project is developed in 3 parts. 1) The hardware for the gimbal. 2) The software to operate the gimbal. 3) Bluetooth/Serial interface for controlling the device with inputs. 

This is a work in progress and has been submitted to the Hackaday Prize. https://hackaday.io/project/11598-open-lidar

The current status of the software is that it is capable of:
- Controlling 2 stepper motors
- Checking a switching hall effect sensor to detect magnet or no magnet
- Writing to a microSD card
- Creating a unique file name on the SD card
- Connecting to a computer via Bluetooth (and outputing serial data)
- Receiving a few commands from a computer/phone over bluetooth
- Reading data from the SF30 module

Future items needed
- Fix scanRoom known bugs
- Add capability for scanRoom to scan at user defined multiple resolutions
- Add filter to prevent scanRoom from over-sampling near polar regions
- Optimize speed of motors
- Add ability to read/write settings to ROM
- Add ability to change default settings
- Synchronize motor timing with SF30 reading
- Create progress indicators and time estimates
- Add ability to save as E57 point cloud file
- Add routine for using hall switch to identify start point and blind spot
- Add ability to create custom filename (i.e. date and time)
- Improve Bluetooth speed and latency
- Statistically evaluate SF30 data
- Create optional per/pitch smoothing algorithm for SF30 data (built in smoothing doesn't start over when switching to a new direction)
- Identify proper warm-up time for sensor
- Rewrite code to leverage the Teensy 3.5 board capabilities.
