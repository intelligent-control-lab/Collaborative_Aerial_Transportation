# Aerial collaborative manipulation

This repository contains a python-ROS-gazebo framework for aerial collaborative manipulation development using UAVs based on the open sourced rotorS project.
This platform is dedicated to be a test bed for the collaborative control strategy developpment. It is centralized in the sense that all the agents communicate with a central control node. It can be decentralized in the sense of assymetric information among the agents.

## Installations
1. Install this package with [ROS indigo](http://wiki.ros.org/indigo/Installation/Ubuntu "ROS indigo") or [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu "ROS kinetic"):
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
$ sudo rosdep init
$ rosdep update
$ source /opt/ros/kinetic/setup.bash
```
2. create your ROS package:
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
```
3. clone the project into the src directory of your workspace:
```bash
git clone https://github.com/intelligent-control-lab/collaborative-aerial-transportation.git
catkin_make
source devel/setup.bash
```

### Launch an example
**Collaborative transportation example**
Here is an example of the centralized collaborative payload transportation task. Four UAVs are physically connected to a box payload via spherical joints. During the example the system expeirence two phases, the hovering and the trajectory following. After steadily hovering, the planner generates a high-order polynomial trajectory that the four UAVs follows simultaneously. Force-torque sensors are attached to the joints between UAVs and the payload. More complecated strategies can be developped based on the current framework.

![collaborative transportation](https://raw.githubusercontent.com/lucasyu17/collaborative_rotorS/master/rotors_gazebo/images/collaborative_transportation.png "collaborative transportation")

```
roslaunch rotors_gazebo collaborative_hovering.launch
cd src/collaborative_rotorS/rotors_gazebo/scripts/collaborative
python mellinger_collaborative_nlopt.py
```

**Single UAV control example**
Here is the example of controlling a single UAV to hover and follow a referenced trajectory.

```
roslaunch rotors_gazebo mellinger_hummingbird.launch
cd src/collaborative_rotorS/rotors_gazebo/scripts/collaborative
python mellinger_trj_nlopt.py
```

## Contents and References
**Position controller for UAV: Mellinger controller.**
This controller is based on the work of D.Mellinger and V.Kumar, and should be cited if this content is used in a scientific publication (or the preceding conference papers):
[1] D. Mellinger and V. Kumar,**b"Minimum snap trajectory generation and control for quadrotors" **, 2011 IEEE International Conference on Robotics and Automation, Shanghai, 2011, pp. 2520-2525.

```
@INPROCEEDINGS{5980409,
author={D. {Mellinger} and V. {Kumar}},
booktitle={2011 IEEE International Conference on Robotics and Automation},
title={Minimum snap trajectory generation and control for quadrotors},
year={2011},
pages={2520-2525},
doi={10.1109/ICRA.2011.5980409},
ISSN={1050-4729},
month={May},}
```

**Minimum snap polynomial trajectory generation**
The trajectory generation part is a python version of the implementation of the reaserch paper:
[2] C. Richter, A. Bry, and N. Roy, **“Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments,”** in International Journal of Robotics Research, Springer, 2016.
A more complete C++ open source version including feasibility verification can also be seen and refers to: [ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation "ethz-asl/mav_trajectory_generation").
```
@incollection{richter2016polynomial,
  title={Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments},
  author={Richter, Charles and Bry, Adam and Roy, Nicholas},
  booktitle={Robotics Research},
  pages={649--666},
  year={2016},
  publisher={Springer}
}
```

**ROS rotors_gazebo environment**
The UAV-payload model is developped based on the existing work of [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator "ethz-asl/rotors_simulator").
This simulator is based on the work of the following research paper:
[3] Furrer, Fadri & Burri, Michael & Achtelik, Markus & Siegwart, Roland. (2016). ** "RotorS – A Modular Gazebo MAV Simulator Framework”  **  
```
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
### Basic usage
The agents are identified by their unique namespace of their ROS message topics. A control node can receive state informations from any agent by subscribing its topic. Agents receive motor speed messages as control input. 

For the generation of polynomial trajectories, key frame vertices have to be given to the planner, including their positions and other constraints on higher order derivatives. The details can be seen in the [example python script](https://github.com/lucasyu17/collaborative_rotorS/blob/master/rotors_gazebo/scripts/collaborative/polynomialTrjNonlinear/Optimizer_nonlinear.py "example python script"). 

Here is an example of a minimum snap polynomial trajectory generation result:
![collaborative transportation](https://raw.githubusercontent.com/lucasyu17/collaborative_rotorS/master/rotors_gazebo/images/trajectory_result.png "trajectory_generation_example")

## License
The MIT License (MIT)
