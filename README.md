# model_train_control
## Overview
This repository contains my project to build software to control a model railway, using a pc and an arduino. The desctiption below will be tailored more towards a model-railway enthusiast than a software engineer.
The idea is that the arduino is connected to the electronics on the railway (track for locomotive power, servo motors to switch points, lights in buildings, etc). The adruino communicates with a pc via the serial interface (usb cable). The software running on the pc is what determines which commands should be sent to the arduino. The pc software also includes a Graphical User Interface (GUI), to provide the user with manual control over the railway. The pc software also allows for fully and semi-autonomous operation. The project assumes simple analog DC operation, where locomotives are NOT equipped with individual chips (not DCC).  

The code is built on top of the ROS middleware, which is meant for robotics development. This is overkill for what this project aims to achieve, but it does provide tools to interface with microcontrollers, sensors, and actuators. This choice was made mostly to learn more about ROS tools.

## Current features:
- Simple locomotive configuration: a simple text (.yaml) file can be edited to specify the starting and cruising power required for the locomotive, as well as the paths to sound files.
- Action sequences: two actions are implemented. Station departure plays a whistle sound, followed by locomotive sound. As the sound files play, the power is ramped up linearly from the start value to the cruising value as specified in the locomotive's configuration file. Station arrival does the opposite.
- Servo motor control for point switching: two point motors can be operated through the UI, but the arduino pin values and servo positions are all still hardcoded and not easily adjustable like the locomotive config files are. 
- Docker container configured to enable easier installation and running of this code. Tested on ubuntu 24. Other linux distros should work, but running on windows would require changes with regards to providing access to the serial port and audio devices of the host pc.

## Features to be added:
- integration of sensors for advanced automated operation (e.g. IR or sonar to automate stoppong at stations, or passing loops);
- add more sounds, both manual and as part of the preset actions;
- make it easy to add more point controls and to configure them without changing any code.


## DISCLAIMER
If you would like to use this project, please link to the model_train_control github repository page so that people may get information about the project should they need it.
Please also note that the settings for various electronics hardware are hard-coded for my own railway. Do not attempt to use the project with your own hardware / railway without understanding what you should change in order to avoid damaging your components. If you would like to use parts of this project, but are not sure how, feel free to ask questions at philip(dot)scales21(at)gmail(dot)com, or in the "issues" tab on the github repository page.
