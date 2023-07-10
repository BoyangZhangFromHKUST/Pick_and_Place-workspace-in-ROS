import sys
import rospy
import math

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from six.moves import input

def gripper_ready():
    rospy.loginfo("Initialized gripper")
    rospy.loginfo("Reset")
    command = outputMsg.Robotiq2FGripper_robot_output()

    command.rACT = 0
    command.rGTO = 0
    command.rATR = 0
    command.rPR = 0
    command.rSP = 0
    command.rFR = 0

    rospy.sleep(1)
    return command

def gripper_activate(command):
    rospy.loginfo("Step 2: Activate")
    
    command.rACT = 1 # Activate
    command.rGTO = 1 # Calls for movement
    command.rSP = 127 # Desired speed
    command.rFR = 170 # Desired force
    rospy.sleep(1)

    return command

def gripper_grasp(val_int, command):
    rospy.loginfo("The gripper grasps to throbot.movee value="+ str(val_int))
    try:
        command.rPR = val_int
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass

    return command

def gripper_open(command):
    rospy.loginfo("The gripper opens")
    command.rPR = 0

    return command
def publisher():
    rospy.init_node("control_2f_gripper")
    rospy.loginfo("The gripper node has been started")


    pub = rospy.Publisher(
        "Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10
    )

    command_gripper_ready = gripper_ready()
    pub.publish(command_gripper_ready)

    command_gripper_activate = gripper_activate(command_gripper_ready)
    pub.publish(command_gripper_activate)

    rospy.loginfo("Initialization is successful")
    rospy.loginfo("==================================")
    rospy.sleep(0.5)

    command_open = gripper_grasp(80, command_gripper_activate)
    pub.publish(command_open)
    rospy.loginfo("Opening the gripper is successful")
    rospy.loginfo("==================================")
    rospy.sleep(1)

    command_grasp = gripper_grasp(120, command_gripper_activate)
    pub.publish(command_grasp)
    rospy.loginfo("The gripper grasped an object")
    rospy.sleep(0.5)

    command_open = gripper_open(command_gripper_ready)
    pub.publish(command_open)
    rospy.loginfo("Opening the gripper is successful")
    rospy.loginfo("==================================")
    rospy.sleep(1)

if __name__ == '__main__':
    publisher()