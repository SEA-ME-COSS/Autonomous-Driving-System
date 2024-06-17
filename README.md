# **ADS Project**

---

# Result

> Insert Video
> 

# Introduction

<table border="0" rules="none">
<tr border="0">
    <td width="280" height="200" align="center">
        <a href="#">
            <img alt="Yocto Logo" src="/media/logo/yocto-logo.png">
        </a>
    </td>
    <td width="280" height="200" align="center">
        <a href="#">
            <img alt="Qt Logo" src="/media/logo/ros2-foxy.png">
        </a>
    </td>
    <td width="350" height="200" align="center">
        <a href="#">
            <img alt="Covesa Logo" src="/media/logo/gazebo_simulator.png">
        </a>
    </td>
</tr>
</table>

This project involves an autonomous driving project using piracer.

We chose ROS2 as the middleware, and the basic structure consists of Perception, Localization, Path Planning, and Control.

To facilitate individual testing, we implemented the virtual environment in Gazebo and also conducted real vehicle tests.

Ultimately, we integrated with [In-Vehicle Infotainment](https://github.com/SEA-ME-COSS/In-Vehicle-Infotainment.git) to combine the autonomous driving system with the distributed embedded system.

# Architecture

<img src=/media/diagram/software-architecture.png alt="software_architecture" width="100%" height="100%"/>

# Setting

## Requirements

### Simulator

You can test two versions. If you want to test ADS in the Simulator, first set up the [Simulation](https://github.com/SEA-ME-COSS/Autonomous-Driving-Simulation) environment.

After that, install dependencies.

- Eigen
    
    ```bash
    sudo apt update	
    sudo apt install libeigen3-dev
    ```
    
- Ceres
    
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev libgoogle-glog-dev libgflags-dev
    sudo apt install libceres-dev
    ```
    
- ompl
    
    ```bash
    sudo apt install libompl-dev
    ```
    
- ros msg apt install
    
    ```bash
    sudo apt update
    sudo apt install ros-foxy-rclcpp
    sudo apt install ros-foxy-nav-msgs
    sudo apt install ros-foxy-geometry-msgs
    sudo apt install ros-foxy-tf2
    sudo apt install ros-foxy-tf2-geometry-msgs
    sudo apt install ros-foxy-example-interfaces
    sudo apt install ros-foxy-std-msgs
    sudo apt install ros-foxy-vision-msgs
    sudo apt install ros-foxy-sensor-msgs
    ```
    

### Real-World

> Hardware Setting
> 

> Software Setting
> 

# Usage

To run in the Gazebo Simulator, launch the simulator as described in [Autonomous-Driving-Simulation](https://github.com/SEA-ME-COSS/Autonomous-Driving-Simulation.git).

```bash
ros2 launch sim sim.launch.py 
```

After building, run the launch file.

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ads_launch simulation_ads.launch.py
```

> Real World Usage
> 

# References

- Yocto Project. (2021). Yocto Project. https://www.yoctoproject.org/
- Qt Project. (2021). Qt Project. https://www.qt.io/
- Raspberry Pi Foundation. (2021). Raspberry Pi. https://www.raspberrypi.org/
- CAN specification documents: http://esd.cs.ucr.edu/webres/can20.pdf
- ROS2 foxy. https://docs.ros.org/en/foxy/index.html
- Gazebo. https://gazebosim.org/home

Shield: [![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

This work is licensed under a
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg