# robot
## Overview
This is "つま先立ち二足歩行（動歩行）ロボット" simulation. 

## Description
- robot.cpp
This is robot simulation using Open Dynamics Engine(ODE) by mongoos. 
The robot has two legs and two hinge joints each legs (sum=4.)

## Usage
### for Mac
1. install ODE
2. clone robot.cpp
3. g++ robot.cpp -lode -ldrawstuff -DdDOUBLE -framework GLUT -framework OpenGL -o robot
4. ./robot

### for Visual Studio 2017 on Windows
1. install ODE
2. copy contents on robot.cpp to sample*.cpp in ode-0.1X/build/vs2010/
3. compile
4. execute .exe in ode-0.1X/lib/
