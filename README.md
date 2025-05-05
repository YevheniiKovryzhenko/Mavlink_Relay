# Mavlink Relay

## Hello there 👋

This a standalone ground station code I have have developed for relaying data through (both ways).
In essence this console-based app is used for all in-lab experiments where Optitrack Motion capture
system is needed. This was primarily used in WSL1 (Ubuntu) environment, but was also deployed and 
tested on arm64 platform (BeagleBone Blue and Black).
The operations is quite simple: you specify your telemetry radio port, mocap port and enable/disable 
data relay to/from local host.

This is being largely phased out by my new UI-based application - [KGroundControl](https://github.com/YevheniiKovryzhenko/KGroundControl.git),
so check it out!

# Installation:
The easiest way to get started in to install [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), install [Debian](https://apps.microsoft.com/detail/9msvkqc78pk6?ocid=webpdpshare) and switch WSL version to 1. Once set up, you will need the following packages:
```bash
sudo apt-get update && sudo apt-get install cmake build-essential git -y
```
You can then clone the repo and compile using Cmake:
```bash
git clone https://github.com/YevheniiKovryzhenko/Mavlink_Relay.git
cd Mavlink_Relay
cmake -S ./ -B ./build
cd build
make
```
You can run Mavlink_Relay app with -h flag to see usage:
```bash
./Mavlink_Relay -h
```

# Contact
If you have any questions, please feel free to contact me, Yevhenii (Jack) Kovryzhenko, at yzk0058@auburn.edu.

# Credit
This work started during my undergraduate research at [ACELAB](https://etaheri0.wixsite.com/acelabauburnuni) at Auburn University, under the supervision of Dr. Ehsan Taheri. 

I would like to express my appreciation to Ella Atkins for providing access to their ground station code.

Check out my more recent autopilot-related works like [PX4 Simulink IO Framework](https://github.com/YevheniiKovryzhenko/PX4_SIMULINK_IO_Framework.git), [RRTV TiltWing](https://github.com/YevheniiKovryzhenko/RRTV_TiltWing.git) 
and [Quadrotor_with_FF_Control](https://github.com/YevheniiKovryzhenko/Quadrotor_with_FF_Control.git). 

