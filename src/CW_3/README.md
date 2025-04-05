Authors: Zijie Chen (zijie-chen.20@ucl.ac.uk)

Video link: [https://www.youtube.com/watch?v=6YOBiV6amBA&t=29s](https://www.youtube.com/watch?v=4Y0EfmrIsWI)

Description: This package forms the base ROS workspace for the coursework of module COMP0129 (https://github.com/COMP0129-UCL/comp0129_s23_robot.git): Robotic Sensing, Manipulation and Interaction.

## Pre-Requisites
```bash
> sudo apt install ros-noetic-franka-ros ros-noetic-libfranka
```
Gazebo physics simluator is also needed (http://gazebosim.org/). This can be installed and then run with:
```bash
> curl -sSL http://get.gazebosim.org | sh
> gazebo
```

## Installation
1. Download this folder into your **your_catkin_workspace/src/** (replace **your_catkin_workspace** to the name of your folder).

2. Build this project through command:  

```bash
> cd your_catkin_workspace
> catkin build
```

## Run Panda robot Gazebo and rviz
```bash
> source devel/setup.bash
> roslaunch cw3_team_4 run_solution.launch
```

## License

LICENSE:  
Copyright (c) 2019-2021 Zijie Chen

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2019-2023 Zijie Chen except where specified
