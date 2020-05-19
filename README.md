# rob6_lego
This is a continuation of the project created by the robotics group 65 at Aalborg University in the fall of 2016 [link](https://github.com/pbtorrild/rob5_lego)
## How to use the package

To enable connection to the robot run:
```bash
roslaunch commander bringup.launch
```

## IMPROVEMENTS FROM LAST SEMESTER
Under here is a list of improvements in order to create a more stable, secure, and user-friendly solution.

**Summery**
The control algorithm required a lot of work in order to improve the
### Structure of the program
To ease the computational power required to run the package, an entirely new control algorithm was written.  
### The camera system
* Major lag was found using rviz to visualize the camera topic. The setup is made to run at 30 fps, this is clearly not the case, making detections of the markers a lot harder.
* The calibration is impossible fast this does point to the average filter not working. This can have multiple causes, either bad tf, not enough detections, a mess-up when storing the tf's or one detection is published multiple times.
* The camera launch file was running too many topics, creating a hardware overflow, causing the aforementioned lag and was fixed by disabling every ability of the camera except depth and color.
```bash
WARNING [140076919019264] (types.cpp:49) set_xu(ctrl=1) failed! Last Error: Input/output error
ERROR [140076919019264] (global_timestamp_reader.cpp:188) Error during time_diff_keeper polling: set_xu(ctrl=1) failed! Last Error: Input/output error
[ WARN] [1589888516.646550671]: Hardware Notification:USB SCP overflow,1.58989e+12,Error,Hardware Error
[ WARN] [1589888518.654559941]: Hardware Notification:USB SCP overflow,1.58989e+12,Error,Hardware Error
[ WARN] [1589888520.671277825]: Hardware Notification:USB SCP overflow,1.58989e+12,Error,Hardware Error
```


### The moveit package

  _the planner often failed and could not do simple motions such as going to the marker after_

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

### The robot describtion package
* Here multiple arrors was found. The robot was standing on a 5 cm offset
