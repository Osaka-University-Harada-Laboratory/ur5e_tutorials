# ur5e_tutorials

ROS package for Universal Robots UR5e tutorial.

## Dependencies

- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

## Installation

1. Install ROS driver  

```
$ cd catkin_ws  
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver  
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot  
$ rosdep update  
$ rosdep install --from-paths src --ignore-src --rosdistro melodic  
$ catkin build  
$ source ../catkin_ws/devel/setup.bash  
```

2. Install URCap on a e-series robot by following [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md)
    - Don't forget the last step of starting the External Control program on the teach pendant ([known issue](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/55)).
    - After [bringup](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md), start the External Control program to establish the connection  

##### Robotiq gripper (option)
1. Install [tool communication](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/setup_tool_communication.md) on universal robot pendant  

2. Download scripts as `$ git clone git@github.com:TechMagicKK/RobotiqHand.git`

3. Test connections as `$ python test_robotiq.py`

## Usage

1. bringup robots  
    `$ roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=XX.XX.XX.XX`
2. execute external control on the pendant  
3. launch moveit  
    `$ roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch`
4. launch rviz  
    `$ roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz`
5. execute tutorial demo  
    `$ roslaunch ur5e_tutorials [demo_name].launch`

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
