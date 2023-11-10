# rerail_stretchit_manipulation
Manipulation stack for Stretch 2

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
python3 test_ik_on_robot.py
```
### Set up a point
Run keyboard teleop:
```
rosrun stretch_core keyboard_teleop
```
