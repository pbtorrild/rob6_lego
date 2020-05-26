# rob6_lego
This is a continuation of the project created by the robotics group 65 at Aalborg University in the fall of 2019 [link](https://github.com/pbtorrild/rob5_lego)
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
To extract data:
* for vision: 
```bash
roslaunch vision_lego data_extractor.launch
```
* for calibration: 
```bash
roslaunch commander data_extractor.launch
```
## Parameters
### Calibration
Diffrent test modes are provided by:
```xml
<param name="test_mode" type="int" value="2" />
```
It can take 3 values; 0 for osolating stick mode, 1 for movement above marker and 2 for stationary camera test.

### Vision
Insentric parameter is provided by:
```xml
<param name="cam_matrix_mode" type="int" value="2" />
```
It can take 4 values; 1 for high-res image using intels calibration, 2 for low-res image using intels calibration, 4 for low-res costum calibration

ArUco corner refinement method:
```xml
<param name="cam_matrix_mode" type="int" value="2" />
```
It can take 3 values; 0 no refinement, 1 for subpixel refinement, 2 contour refinement

Set marker width in meters:
```xml
<param name="marker_width" type="double" value="0.0388" />
```

### Data extractor
Set sample size:
```xml
<param name="sample_size" type="int" value="1000" />
```
Set file-path:
```xml
<param name="file_path" type="str" value="/home/peter/Desktop/" />
```

Set file-name:
```xml
<param name="file_name" type="str" value="new_alg_test1.csv" />
```

Set separator between columns:
```xml
<param name="separator" type="str" value=";" />
```
