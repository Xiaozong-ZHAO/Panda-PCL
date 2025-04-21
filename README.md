## Panda & PCL Grasp and Place

![fk](/src/cw2_team_35/assets/demo.gif)

The robot first scans the entire scene and builds a global point‑cloud map. After filtering out baskets and obstacles, it examines the remaining objects to determine which shape appears most frequently. Finally, it randomly selects one object matching that dominant shape, grasps it, and places it.

## Pre-Requisites

We are running this project on the OS platform and versions of ROS as below:
* Ubuntu 20.04.6 LTS
* ROS Distribution: noetic
* ROS Version: 1.17.0
* Framework Used: MoveIt
* Gazebo Version: 11.15.1

Based on the repository we worked on, please try to install ROS1, Gazebo, and related packages via official websites and with the command below:

```bash
sudo apt install ros-noetic-franka-ros ros-noetic-libfranka
```
The Gazebo physics simulator is also needed (http://gazebosim.org/). This can be installed and then run with:

```bash
curl -sSL http://get.gazebosim.org | sh
gazebo
```

## Installation

Please complete the installation and follow the steps below in order to run the coursework task:

1. Clone the repository to your desired local directory:

```bash
git clone https://github.com/surgical-vision/comp0250_s25_labs.git --recurse-submodules
```

2. Replace the folder `cw1_team_x` to our `cw1_team_35` under the directory `~/comp0250_s25_labs/src`.

3. Change the directory to the root of the folder and build the workspace with Catkin tools:

```bash
cd comp0250_s25_labs
```

```bash
catkin build
```

## Run Panda Robot Gazebo and RViz
After the build, if all the packages are built successfully, we can continue to initialise the simulation enviornment by running the commands below:

```bash
source devel/setup.bash
```

```bash
roslaunch cw2_team_35 run_solution.launch
```

## Run the Solutions of each Task

To execute the tasks, open a new terminal and run the commands below:

```bash
source devel/setup.bash
```

Then, call the service `rosservice call /task X`, X is number of the task. In order, the three tasks can be run one by one once the previous task is compelted.

```bash
rosservice call /task 1
```
```bash
rosservice call /task 2
```
```bash
rosservice call /task 3
```

## Authors and Related Information

Author: Xiaozong Zhao (Albert Zhao)

The original repository source we worked on: https://github.com/surgical-vision/comp0250_s25_labs, authored by Eddie Edwards
(eddie.edwards@ucl.ac.uk), Kefeng Huang, Bowie (Heiyin) Wong, Dimitrios Kanoulas, Luke Beddow, Denis Hadjivelichkov

Note: This package forms the base ROS workspace for the module COMP0250 (formerly COMP0129): Robotic Sensing, Manipulation and Interaction.

## Licence

This repository is a derivative work based on the original project by Dimitrios Kanoulas, Eddie Edwards, and collaborators, and is licenced under the [MIT Licence](LICENSE.txt).

You are free to use, modify, and distribute this software according to the terms of the MIT Licence. All modifications and contributions by Group 35 are integrated into this repository and are also provided under the same licence.

**DISCLAIMER:** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED. IN NO EVENT SHALL THE ORIGINAL AUTHORS OR GROUP 35 BE LIABLE FOR ANY DAMAGES ARISING FROM THE USE OF THIS SOFTWARE.

© 2019-2024 Dimitrios Kanoulas and Eddie Edwards, with modifications © 2025 Group 35 from UCL COMP0250: Robotic Sensing, Manipulation and Interaction [T2] 24/25, with the members: Albert Zhao
