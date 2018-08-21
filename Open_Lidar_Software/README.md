# OpenLidar
Open Lidar

The idea of this project is to use cheap laser distance modules to create 3D scans of the room. This can be done by rotating the laser distance module on 2 axis with the sensor centered on the junction of those two axis (aka: the sensor is on the center of rotation so it's always at the origin). The main goals of this project are to improve the means that cavers use to survey and map caves as citizen scientists, or to create point-clouds for your own purpose.

The current status of the software is that it is capable of:
- Controlling 2 stepper motors (upgraded since last model)
- Receiving commands and outputing results over serial
- Receiving a few commands from a computer/phone over bluetooth
- Creating a unique file name on the SD card

Future items needed
- Interfacing over bluetooth
- Reading data from the Lidar Lite V3 module
- Writing to a microSD card
- Create a new scan room function
- Add filter to prevent scanRoom from over-sampling near polar regions
- Add ability to read/write settings to ROM
- Add ability to change default settings
- Create progress indicators and time estimates
- Add ability to save as E57 point cloud file
- Add routine for identifying 0 degrees
- Add ability to create custom filename (i.e. date and time)
