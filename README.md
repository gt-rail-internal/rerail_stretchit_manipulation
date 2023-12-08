# rerail_stretchit_manipulation

## Package Description

This package's primary component is `manipulation_grasp.py`, a ROS Noetic service utilized by the behavior tree of the stretch stack. It operates with a custom message defined in `RerailManip.srv` located in the `srv` folder. The service accepts a `geometry_msgs/Pose` `target_pose` as input and returns a Boolean message upon completing the manipulation task. The manipulation process employs the `StretchIK` class from `src/manipulation/ik.py`. This class computes the inverse kinematics for the provided pose message to derive the joint values necessary to reach the targeted object. This script is inspired by Hello Robot's [Inverse Kinematics Tutorial Workshop](https://forum.hello-robot.com/t/inverse-kinematics-tutorial-workshop-recording/639) and a [previous implementation](https://github.com/mlamsey/stretch_ik) by mlamsey.

### Other Important Files:

- `manipulation_delivery.py`: A service for executing predefined trajectories to place a grasped object on a surface, returning a Boolean message.
- `move_camera`: A service for repositioning the robot's camera to a predefined location, enhancing object visibility for grasping.
- `urdf_ripper.py`: Adapted from [mlamsey's stretch_ik repository](https://github.com/mlamsey/stretch_ik), this script streamlines the robot's original URDF file by eliminating unnecessary links, generating a new, optimized version.

### Inputs and Outputs (ROS msg types)
- `manipulation_grasp.py`: 
    - Input: `geometry_msgs/Pose` / Output: `std_msgs/Bool.msg`
- `manipulation_delivery.py`: 
    - Input: `None` / Output: `std_msgs/Bool.msg`
- `move_camera`: 
    - Input: `None` / Output: `std_msgs/Bool.msg`

## Generating a URDF for ikpy

The `/urdf` folder contains a simplified URDF file for ikpy usage. To regenerate it (for instance, after calibrating your Stretch URDF), follow these instructions:

1. Navigate to `/src` using `cd`.
2. Run `python urdf_ripper.py --save_path /path/to/output/urdf/stretch.urdf`.
3. Move the generated URDF file to `/urdf` and ensure it is named `stretch.urdf`.

The `urdf_ripper.py` script discards all extraneous links and joints from the original URDF and saves the streamlined version to the specified output path. The script is based on a tutorial notebook.

## Robot Coordinate System

For ease of inverse kinematics calculations, the robot is treated as a Cartesian manipulator. Here, the x-axis represents base translation, the y-axis denotes arm extension, and the z-axis signifies the height of the robot's arm. The robot must always be perpendicular to the surface from which it intends to grasp objects.

## Tips

Review the hardcoded values in `manipulation_grasp.py` and `manipulation_delivery.py`. While these values are generally effective for most surfaces, it's crucial to test them in your specific setup to prevent potential collisions.

## Dependencies 

This package depends on `ikpy`. You can install it using the following command:

```bash 
pip3 install -q ikpy
