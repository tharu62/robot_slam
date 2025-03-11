# HOW TO MAKE A ROBOT FOR SLAM

## Table OF Contents
```bash
- Project Description...................................................... .
  - Brief ................................................................. .
  - Software .............................................................. .
    - OS .................................................................. .
    - ROS ................................................................. .
    - Code and Architecture ............................................... .
  - Hardware .............................................................. .
    - Linux Machine ....................................................... .
    - Microcontrollers .................................................... .
    - Sensors ............................................................. .
        - Lidar ........................................................... .
        - Encoders ........................................................ .
    - Actuators ........................................................... .
        - DC Motors ....................................................... .
- Conclusions ............................................................. .
- Errors .................................................................. .
- WSL Options ............................................................. .
```

## Project Description

### BRIEF
This repository contains a detail description of how to build and set up a differential drive robot that can perform SLAM (Simultaneous Localization and Mapping). The 

### SOFTWARE
### OS 
The Operating System used for development and to compute the SLAM is Ubuntu 24.04 LTS. Note that this specific project was developed with WSL2 from a Windows 11 machine and no particular issue was found (more on the last pargraph of the README).

### ROS 
The Robot Operating System (ROS) distro used is Jazzy Jalisco. No particular issue was found in the set up of ROS with Ubuntu 24.04 and the code could also teorically run on ROS Humble (not reliably). 
### Code and Architecture 
Workspace...
Package info and git clone command...
Nodes description...
Launch file descriptions and use...

### HARDWARE 
### Linux Machine 
### Microcontrollers 
### Sensors 
### Lidar
### Encoders
### Actuators 
### DC Motors

## Conclusion

## Errors

## WSL options
if rviz2 presents a graphic glitch in wsl2 it is probably due to grafix probelms: add to ~/.bashrc

```bash
export LIBGL_ALWAYS_INDIRECT=0

export LIBGL_ALWAYS_SOFTWARE=1
```
