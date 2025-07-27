# ET-GOA Sim + Live Study

## Overview
This project is for the evaluation of the Event-Triggered Generalized Outcome Assessment (ET-GOA) algorithm using both
a simulated and live robot.

ET-GOA leverages the Generalized Outcome Assessment (GOA) and Model Quality Assessment (MQA) metrics from Factorized
Machine Self-Confidence (FaMSeC) to enable an autonomous robot to understand when and how its competency changes during
task execution. This project extends the following:

[Dynamic Competency Self-Assessment for Autonomous Agents](https://arxiv.org/abs/2303.01646)

We published this paper: 

[Event-triggered robot self-assessment to aid in autonomy adjustment](https://www.frontiersin.org/articles/10.3389/frobt.2023.1294533/full)


### Please cite us if you use this code:

Conlon N, Ahmed N and Szafir D (2024), Event-triggered robot self-assessment to aid in autonomy adjustment. Front. Robot. AI 10:1294533. doi: 10.3389/frobt.2023.1294533


```
@ARTICLE{10.3389/frobt.2023.1294533,
    AUTHOR={Conlon, Nicholas  and Ahmed, Nisar  and Szafir, Daniel },
    TITLE={Event-triggered robot self-assessment to aid in autonomy adjustment},
    JOURNAL={Frontiers in Robotics and AI},
    VOLUME={Volume 10 - 2023},
    YEAR={2024},
    URL={https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2023.1294533},
    DOI={10.3389/frobt.2023.1294533},
    ISSN={2296-9144}
}
```



## Installation
This was tested on Ubuntu 20.04 using python 3.8.x

### Install ROS
Install ROS Noetic following the instructions [here](https://wiki.ros.org/noetic/Installation/Ubuntu). 

I also have the Clearpath Jackal ROS packages installed, you can follow the installation
instructions [here](https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html) or run:
```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```

### Install Webots:
Install webots R2023.a from [here](https://www.cyberbotics.com/#download)


### Install python libraries
Install the python libraries for this project by running:
```
python -m pip install -r requirements.txt
```

## Usage
Make sure ROS is built. Follow the instructions [here](https://wiki.ros.org/noetic/Installation/Ubuntu)
to set up a catkin workspace, then run:
```commandline
catkin build
```

Start Gazebo. I use the Clearpath Jackal robot: 
```commandline
$ roslaunch jackal_gazebo empty_world.launch
```

Start the user interface:
```commandline
$ python3 ./interface/ros_ui_main.py
```

The interface should be started. Telemetry should be flowing. Entering a goal in the (x,y) boxes will update the
goal. Plans are generated using RRT. Auto starts autonomous navigation, stop stops navigation, and telop waits for
telop commands. Robot competency can be seen on the right panel. Below are several buttons to turn on/off different
functions and end the simulation.

