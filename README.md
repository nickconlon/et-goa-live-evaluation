# ET-GOA Sim + Live Study

This project is for the evaluation of the Event-Triggered Generalized Outcome Assessment (ET-GOA) algorithm using both
a simulated and live robot.

ET-GOA leverages the Generalized Outcome Assessment (GOA) and Model Quality Assessment (MQA) metrics from Factorized
Machine Self-Confidence (FaMSeC) to enable an autonomous robot to understand when and how its competency changes during
task execution. This project extends the following:

[Dynamic Competency Self-Assessment for Autonomous Agents](https://arxiv.org/abs/2303.01646)

We published this paper: 

[Event-triggered robot self-assessment to aid in autonomy adjustment](https://www.frontiersin.org/articles/10.3389/frobt.2023.1294533/full)

### Installation
Python: 3.8.10 + libs in requirements.txt

ROS: noetic

Webots: R2023a

### Usage
Start some ROS environment if this is for simulation (I use the base Jackal): 
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

