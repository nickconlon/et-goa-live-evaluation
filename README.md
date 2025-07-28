# ET-GOA Sim + Live Study

## Overview
This project is for the evaluation of the Event-Triggered Generalized Outcome Assessment (ET-GOA) algorithm using both
a simulated and live robot.

ET-GOA leverages the Generalized Outcome Assessment (GOA) and Model Quality Assessment (MQA) metrics from Factorized
Machine Self-Confidence (FaMSeC) to enable an autonomous robot to understand when and how its competency changes during
task execution. This project extends our previous work:

[Generalizing Competency Self-Assessment for Autonomous Vehicles Using Deep Reinforcement Learning](https://arc.aiaa.org/doi/10.2514/6.2022-2496)

[Dynamic Competency Self-Assessment for Autonomous Agents](https://arxiv.org/abs/2303.01646)

We published this article: 

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

#### The interface runs everything:
The left panel shows telemetry. Middle left allows user to (1) set a goal location for
the robot to drive to, (2) generate a plan to that goal, and (3) select a level of autonomy for the robot to
operate at.

The center panel shows of map with the robot's location (blue), goal (green), and waypoints (black). The GOA predictions
will be plotted as an orange circle to visualize how well the World Model predictions align with real state
operations during the task.

The right panel shows the Competency Self-Assessment computed using Factorized Machine Self-Confidence for four
different factors:
- Generalized Outcome Assessment (GOA): A metric of the robot's ability to autonomously achieve a task outcome.
- Model Quality Assessment (MQA): A metric of alignment between the robot's World Model and reality.
- Solve Quality Assessment: Not implemented.
- Experience Assessment: Not implemented.

For this experiment we just looked at the outcome space of "time to complete the task". GOA is a metric
of the robot's ability to complete the task within some time threshold, which can be set.

MQA below some threshold is used to trigger a re-assessment of GOA given whatever new information is available
to the robot's autonomy and World Model.

#### To run the experiments:
- Enter a new goal (x,y) within the mission area (black box on plot).
- Press `Generate Plan`. A waypoint plan should display from the robot to the goal.
- Press `FaMSeC on`. This enables the GOA/MQA components.
- Enter an outcome threshold and press `Set outcome threshold (sec)` button.
- Press `Run new Outcome Assessment`. This (1) kicks off the World Model to simulate the robot executing the waypoint
plan (and save output to `/rollouts`), (2) computes GOA, (3) displays the GOA confidence label. This is an estimate of the robot's confidence in
achieving the desired task time threshold. User can select different times and rerun the assessment.
- If GOA is within users desired level of risk, press `Auto` to command the robot to autonomously navigate to the
goal along the waypoints.
- While in Auto mode, the Model Quality Assessment is computed in realtime as a metric of the robot's observed state
and the robot's World Model predictions. The MQA is reported using the same semantic labels as GOA. The combination of
GOA assessment and MQA triggering in known as *Event-Triggered Generalized Outcome Assessment*.
- With the `Triggering on` button enabled, the robot will hault operation if the MQA gets too low. This indicates
significant deviation between what the robot's autonomy predicted and what the robot has observed.


#### Other things (that I have really just to make testing easier):
- `Go home` routes the robot back to home (2,2).
- `Test obstacle` places a random obstacle in front of the robot, `Clear obs` removes it.
- `Test trigger` is an in progress mechanism to test the MQA trigger.
- User can also record the data (`Record on`) and write it out (`Write data`) to a data.csv file in the directory root.


#### Limitations:
The World Model itself was not the primary focus of this project, so we used a simple digital twin representation
of the robot in Webots. The World Model robot generally emulates the Jackal we used, however the environment is quite
simple. We can dynamically add obstacles or change the ground plan to increase/decrease friction. However, we found that
it's not a great analogue for the real-world. In particular the World Model has difficulty modeling the
robot turning and changing speed, which leads to generally lower MQA scores.

I encourage anyone interested to create their own World Model that follows the interface definition outlined in
`\interface\world_model.py`. We have conducted limited experiments with the following as World Model:
- Gazebo simulation.
- Webots simulation.
- Monte-Carlo Tree Search (MCTS) with an appropriate policy over some simple graphs.
- A handful of environments from OpenAI Gym.






