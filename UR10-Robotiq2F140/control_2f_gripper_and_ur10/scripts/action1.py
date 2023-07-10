import rospy
from gripper_basic_functions_ import gripper
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

g = gripper()

def gripper_move():
    rospy.init_node("control_2f_gripper")
    rospy.loginfo("Gripper Initialization")

    pub = rospy.Publisher(
        "Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10
    )

    gripper_ready = g.gripper_ready()
    pub.publish(gripper_ready)

    gripper_activate = g.gripper_activate()
    pub.publish(gripper_activate)

    rospy.sleep(15)

    rospy.loginfo("Initialization is successful")
    rospy.loginfo("==================================")
    rospy.sleep(2.5)

    up = g.up_threshold()
    down = g.down_threshold()

    for i in range(down, up, 10):
        gripper_grasp = g.gripper_grasp(i)
        pub.publish(gripper_grasp)
        rospy.sleep(0.1) # the frequency of grasping


    rospy.sleep(0.5)

    gripper_close = g.gripper_close()
    pub.publish(gripper_close)
    rospy.sleep(0.5)


if __name__ == '__main__':
    gripper_move()  