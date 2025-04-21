# UCL MSc Robotics and Artificial Intelligence COMP0250 Coursework 1
## Authors and Related Information

Group number: 35

Authors: Jiaheng Wang, Yu-Chun Lo, Albert Zhao

The original repository source we worked on: https://github.com/surgical-vision/comp0250_s25_labs, authored by Eddie Edwards
(eddie.edwards@ucl.ac.uk), Kefeng Huang, Bowie (Heiyin) Wong, Dimitrios Kanoulas, Luke Beddow, Denis Hadjivelichkov

Note: This package forms the base ROS workspace for the module COMP0250 (formerly COMP0129): Robotic Sensing, Manipulation and Interaction.

## Licence

This repository is a derivative work based on the original project by Dimitrios Kanoulas, Eddie Edwards, and collaborators, and is licenced under the [MIT Licence](LICENSE.txt).

You are free to use, modify, and distribute this software according to the terms of the MIT Licence. All modifications and contributions by Group 35 are integrated into this repository and are also provided under the same licence.

**DISCLAIMER:** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED. IN NO EVENT SHALL THE ORIGINAL AUTHORS OR GROUP 35 BE LIABLE FOR ANY DAMAGES ARISING FROM THE USE OF THIS SOFTWARE.

© 2019-2024 Dimitrios Kanoulas and Eddie Edwards, with modifications © 2025 Group 35 from UCL COMP0250: Robotic Sensing, Manipulation and Interaction [T2] 24/25, with the members: Jiaheng Wang, Yu-Chun Lo, Albert Zhao 

## Code Usage

Please complete the installation and follow the steps below in order to run the coursework task:

1. Replace the folder `cw2_team_x` to our `cw2_team_35` under the directory `path_to_ws/comp0250_s25_labs/src`.

2. Change the directory to the root of the folder and build the workspace with Catkin tools:

```bash
cd comp0250_s25_labs
```

```bash
catkin build
```

3. To execute the solutions, please run the commands below:

```bash
source devel/setup.bash
```

```bash
roslaunch cw2_team_35 run_solution.launch
```

Then, open another terminal, and run the following commands:

```bash
source devel/setup.bash
```

For task 1, run:
```bash
rosservice call /task 1
```

For task 2, run: 
```bash
rosservice call /task 2
```

For task 3, run: 
```bash
rosservice call /task 3
```

## Potential Failure cases:
### Please make sure T2_GROUND_PLANE_NOISET would not affect t1 and t3
1. After calling task 2 service, if task 3 or task 1 service are called, the world_spawner does not delete the previously spawned grass tile. When the T2_GROUND_PLANE_NOISET is activated, this can leads to model interpenetration or jamming with the existing grass tiles and ultimately causes the task to fail. Even if no jamming occurs, the Task 2 scene setup—particularly the height‑perturbed tiles—still skews the passthrough filter in Task 1 and Task 3, causing those subsequent tasks to be judged as failures.

![fk](src/cw2_team_35/figures/bug.jpg)

### Gripper problem
2. Occasionally, when the Panda arm moves along a long curve path, the gripper sometimes opens unexpectedly though it never receives a release command, causing the object to slip out during the motion.

### Task 1:

## Total Time and Effort Distribution

### Task 1:
- **Time Taken:** Roughly 18 hours
- **Task Allocation:**  
  - Albert Zhao: 33% (PCL part)
  - Jiaheng Wang: 33% (Robot arm planning part)
  - Yu-Chun Lo: 33% (Worked with T1_ANY_ORIENTATION = True)

### Task 2:
- **Time Taken:** Roughly 12 hours
- **Task Allocation:**  
  - Albert Zhao: 33% (PCL: Mainly SHOT descriptor based classification)
  - Jiaheng Wang: 33% (PCL: Mainly Centroid‑based classification and T2_ANY_ORIENTATION)
  - Yu-Chun Lo: 33% (Worked with T2_GROUND_PLANE_NOISE = 50e-3)

### Task 3:
- **Time Taken:** Roughly 30 hours
- **Task Allocation:**  
  - Albert Zhao: 33% (Mainly Scene scanning and octomap)
  - Jiaheng Wang: 33% (Fully revised the planning strategy for larger range movement)
  - Yu-Chun Lo: 33% (Worked with obstacle avoidance)