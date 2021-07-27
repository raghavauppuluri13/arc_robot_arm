# arc_robot_arm

Welcome to the ARC Robot Arm project repo!

Our overarching goal is to explore the robotic manipulation, robot vision, robot control, and reinforcement learning by building a robot arm. In our journey, we plan to publish our progress, tutorials, and understandings. Learn more on the [wiki](https://wiki.purduearc.com/wiki/robot-arm/start-here).

## Setup 

1. [Setup ROS](https://wiki.purduearc.com/wiki/tutorials/setup-ros) if you haven't done so.

2. Clone this repo into the `src` folder in your ROS workspace recursively to get the `yolov5_pytorch_ros` and `realsense_ros_gazebo` packages
```
git clone --recursive https://github.com/purdue-arc/arc_robot_arm.git
```
3. Download all package dependencies / Setup environment

**Non-Robostack**:
```
cd path/to/root_workspace_dir
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```
**Robostack Only**:
```
# Ensure base conda environment is activated
cd arc_robot_arm
mamba env create -f robot_arm_environment.yml
conda activate robot-arm-env
```
4. Build + source (Do this every time you download new packages)
```
catkin build
source path/to/catkin_ws/devel/setup.bash
```
> Note: Build + source every time you add new packages. Source every time you open a fresh terminal, or add the line to your ~/.bashrc so it sources automatically

## Next Steps

### To get right to the code 
- Launch the robot in Gazebo simulation with the chessboard world and Realsense ROS gazebo camera, using the [protoarm_bringup](https://github.com/purdue-arc/arc_robot_arm/tree/main/protoarm_bringup) package 
- Use the IK to run some test goal positions using the [protoarm_kinematics](https://github.com/purdue-arc/arc_robot_arm/tree/main/protoarm_kinematics) package 
- Launch the chess piece detector which uses a YOLOv5 detection model, using the [chess_piece_detector](https://github.com/purdue-arc/arc_robot_arm/tree/main/chess_piece_detector) package 

### For a more verbose and human-readable explanation of what we're doing 

Check out the wiki pages [here](https://wiki.purduearc.com/wiki/robot-arm/start-here) to learn more about all the different components and how they fit together.

## Questions or problems?

Add an [issue](https://github.com/purdue-arc/arc_robot_arm/issues/new/choose) or add comments under articles in the wiki and we'll respond ASAP!
