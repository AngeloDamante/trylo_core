# Vision Control System with Ros2 on Trilobot
Ros2 core for Trilobot© by [Pimoroni](https://shop.pimoroni.com/products/trilobot), renamed `trylo` for this context. 
The **goal** is to implement a control system based on aruco detection and improved acquisition with Kalman filter.

<p align="center">    
    <img src=doc/trylo_logo_img.jpg  alt="logo"/>
</p>

## Directories Layout
The core consists of 3 main packages. `vision`, `gpio` and `control`. In addition, the package `launch` 
was created to launch nodes by choosing the working context.
```
trylo_core
├── src
│   ├── definitions.py      # GLOBAL
│   ├── parameters.py       # GLOBAL
│   ├── trylo_control       
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── msg
│   │   │   ├── Led.msg
│   │   │   └── Reference.msg
│   │   ├── scripts         
│   │   │   ├── __init__.py
│   │   │   ├── n_control.py
│   │   │   ├── n_refgen.py
│   │   │   └── n_tst_ref.py
│   │   └── trylo_control
│   │       ├── __init__.py
│   │       ├── utils.py
│   │       └── VisionKalmanFilter.py
│   ├── trylo_gpio
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── msg
│   │   │   └── Command.msg
│   │   ├── scripts
│   │   │   ├── __init__.py
│   │   │   ├── n_pub_cmd.py
│   │   │   └── n_robot.py
│   │   └── trylo_gpio
│   │       ├── __init__.py
│   │       ├── names.py
│   │       └── Trylo.py
│   ├── trylo_launch
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── launch
│   │   │   ├── test_control.xml
│   │   │   ├── test_motors.xml
│   │   │   ├── test_refgen.xml
│   │   │   ├── vision_control.xml
│   │   │   └── vision_nodes.xml
│   │   └── trylo_launch
│   │       └── __init__.py
│   └── trylo_vision
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include
│       │   └── trylo_vision
│       ├── msg
│       │   ├── Marker.msg
│       │   ├── MarkersDetected.msg
│       │   └── Pair.msg
│       ├── scripts
│       │   ├── __init__.py
│       │   ├── n_aruco_detector.py
│       │   ├── n_aruco_drawer.py
│       │   └── n_camera.py
│       └── trylo_vision
│           └── __init__.py
├── Makefile
├── start_trylo.py
├── start_script.sh
└── README.md
```

## Installation
Requirements:
```
- ros2 >= foxy
- python >= 3.8
- opencv >= 4.7
- scipy >= 1.10
```
In the home vehicle directory, let's install project,
```
git clone https://github.com/AngeloDamante/trylo_core.git

# optionally install rosboard 
cd src
git clone https://github.com/dheera/rosboard.git
```
setting up the environment variables
```
echo "TRYLO_CORE=~/trylo_core" >> ~/.bashrc
echo "PYTHONPATH=${PYTHONPATH}:${TRYLO_CORE}/src" >> ~/.bashrc
echo "sh ${TRYLO_CORE}/start_script.sh" >> ~/.bashrc
```

[//]: # (script per installare tutto)
[//]: # (script per installare tutto con flag di rosboard)

## Architecture
<p align="center">    
    <img src=doc/Trylo_ros2_scheme.jpg  alt="architecture"/>
</p>
Let's take a quick overview of the ros2 packages:

- `trylo_control`: these nodes implement control system, the `refgen` node generates references and `control` node provides commands to the robot.
- `trylo_gpio`: this node directly interfaces to the vehicle using `Trylo` library.
- `trylo_vision`: these nodes process the images acquired by the PiCamera and detects the aruco marker. 

### Control System
The control system is based on the markers that are detected. There are `ENABLE_MARKER` and `DISABLE_MARKER` to enable and disable control system.
Furthermore, there are Targets as `TARGET_00`, `TARGET_01`, and so on. 

Reference generation is handled by the following finite state machine.
<p align="center">    
    <img width=600 src=doc/asf.jpg  alt="architecture"/>
</p>

You can set the control system using `parameters.py` file in main directory.
```
DEG_RANGE = (-5.0, 5.0)
D_MIN = 0.2
D_MAX = 2.0
SPEED_MIN = 0.6
SPEED_MAX = 1.0
KF_ENABLE = True
MAX_LOST_FRAME = 20    # camera connection safeguard
MIN_KF_SAMPLES = 10    # to consider the filter in ready state
```

If you are interested in learning more about the Kalman filter used, 
you can see this [repo](https://github.com/AngeloDamante/pose-estimation-kalman).

## Using
You have two ways to launch and use this control system. 

### Using Launch file
This project provides launch package `trylo_launch` to simplify starting of control system.
```
cd trylo_core

# build
colcon build        # for the first time
make build_launch   # otherwise

# and now we can launch control system
source install/setup.bash
ros2 launch trylo_launch vision_control.xml
```

In addition, if you wish, you can use rosboard to view topics as camera.
```
ros2 run rosboard rosboard_node
```

### Using start script in bash file
If, instead, you want the control system to start directly when the vehicle starts, 
you can include the script in the bash file.

```
echo "sh ${TRYLO_CORE}/start_script_cs.sh" >> ~/.bashrc
```