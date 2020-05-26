# rob6_lego
This is a continuation of the project created by the robotics group 65 at Aalborg University in the fall of 2016 [link](https://github.com/pbtorrild/rob5_lego)
## How to use the package
### Launch files
To enable connection to the robot run:
```bash
roslaunch commander bringup.launch
```
To enable pathplanning: 
```bash
roslaunch lego_moveit execute.launch
```
To enable vision: 
```bash
roslaunch vision_lego all.launch
```
To enable calibration:
```bash
roslaunch commander control.launch
```
