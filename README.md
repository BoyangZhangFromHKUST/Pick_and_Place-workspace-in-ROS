# Pick_and_Place-workspace-in-ROS

## Connect UR10 with ip:

`roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.102`

## Execute motion planning:

`roslaunch ur10_moveit_config moveit_planning_execution.launch`

## Connect Robotiq Gripper with RTU communication with USB port:

`rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB1`

## Publish the node of the tactile sensor:

`rosrun tactile_sensor class_touch_edit.py`

## Waiting for subscription:

`rosrun tactile_sensor ur10_demo.py`

## Control the gripper moving:

`rosrun control_2f_gripper_and_ur10 action1.py`

## Description:

After the gripper moves, the tactile sensor will be closing to the object. After the tactile sensor touches the object, the `class_touch_edit.py` will publish the topic to the node. And `ur10_demo.py` will subscribe to the topic from the node and tell the robot how to make the next action.
