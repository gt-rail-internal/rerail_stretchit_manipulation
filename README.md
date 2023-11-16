# rerail_stretchit_manipulation
Manipulation stack for Stretch 2:

This package uses the `StretchIK` class to compute the inverse kinematics to grasp an object given a 3D pose. It considers Stretch as a cartesian manipulator.
The script `python3 manipulation_grasp.py` executes a series of commands to grasp an object using a top-down approach.

### How to use it
In one terminal run:
```
roslaunch stretch_core stretch_driver.launch
```
If you want to use Rviz run this in another terminal:
```
rosrun rviz rviz -d `rospack find stretch_core`/rviz/stretch_simple_test.rviz
```
Finally, run the test node inside the scripts folder:

```
python3 manipulation_grasp.py
```
### You need grasp pose working!
TODO:
Put this together with the grasp pose package.
