# rob6_lego
This is a continuation of the project created by the robotics group 65 at Aalborg University in the fall of 2016 [link](https://github.com/pbtorrild/rob5_lego)
## How to use the package

To enable connection to the robot run:
```bash
roslaunch commander bringup.launch
```

## IMPROVEMENTS FROM LAST SEMESTER
Under here is a list of improvements in order to create a more stable, secure, and user-friendly solution
### Structure of the program

### The camera system
* Major lag was found using rviz to visualize the camera topic. The setup is made to run at 30 fps, this is clearly not the case, making detections of the markers a lot harder.



### The moveit package
* Planner was not able to go to the marker once it was found
```bash
[ WARN] [1589885494.085382272]: "marker_0" passed to lookupTransform argument source_frame does not exist.
[ WARN] [1589885494.285719129]: "marker_0" passed to lookupTransform argument source_frame does not exist.
[ INFO] [1589885497.234707158]: Found marker with id: 0
[ WARN] [1589885502.268401016]: Fail: ABORTED: No motion plan found. No execution attempted.
[ INFO] [1589885507.283054102]: ABORTED: No motion plan found. No execution attempted.
[ INFO] [1589885507.283112595]: Starting Calibration (Part 1 of 2)
[ INFO] [1589885509.783295402]: Starting Calibration (Part 2 of 2)
[ INFO] [1589885512.283452384]: Calibrating...
[ WARN] [1589885517.343545003]: Fail: ABORTED: No motion plan found. No execution attempted.
[ INFO] [1589885522.372203112]: ABORTED: No motion plan found. No execution attempted.

```

### The Robot describtion package
