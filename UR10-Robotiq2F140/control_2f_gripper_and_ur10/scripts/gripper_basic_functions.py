# the copy file has gone to devel/lib/control_2f_gripper_and_ur10 as gripper_basic_functions_.py
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class gripper(object):
    def gripper_ready(self):
        rospy.loginfo("Initialized gripper")
        rospy.loginfo("Reset")
        self.command = outputMsg.Robotiq2FGripper_robot_output()

        self.command.rACT = 0
        self.command.rGTO = 0
        self.command.rATR = 0
        self.command.rPR = 0
        self.command.rSP = 0
        self.command.rFR = 0

        rospy.sleep(1)
        return self.command
    
    def gripper_activate(self):
        rospy.loginfo("Step 2: Activate")
    
        self.command.rACT = 1 # Activate
        self.command.rGTO = 1 # Calls for movement
        self.command.rSP = 127 # Desired speed
        self.command.rFR = 170 # Desired force
        rospy.sleep(1)

        return self.command
    
    def gripper_grasp(self,val_int):
        rospy.loginfo("The gripper grasps to throbot.movee value="+ str(val_int))
        try:
            self.command.rPR = val_int
            if self.command.rPR > 255:
                self.command.rPR = 255
            if self.command.rPR < 0:
                self.command.rPR = 0
        except ValueError:
            pass

        return self.command
    
    def gripper_open(self):
        rospy.loginfo("The gripper opens")
        self.command.rPR = 0

        return self.command
    def publisher(self):
        rospy.init_node("control_2f_gripper")
        rospy.loginfo("The gripper node has been started")


        pub = rospy.Publisher(
            "Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10
        )

        command_gripper_ready = self.gripper_ready()
        pub.publish(command_gripper_ready)

        command_gripper_activate = self.gripper_activate()
        pub.publish(command_gripper_activate)

        rospy.loginfo("Initialization is successful")
        rospy.loginfo("==================================")
        rospy.sleep(0.5)

        command_open = self.gripper_grasp(80)
        pub.publish(command_open)
        rospy.loginfo("Opening the gripper is successful")
        rospy.loginfo("==================================")
        rospy.sleep(1)

        command_grasp = self.gripper_grasp(120)
        pub.publish(command_grasp)
        rospy.loginfo("The gripper grasped an object")
        rospy.sleep(0.5)

        command_open = self.gripper_open()
        pub.publish(command_open)
        rospy.loginfo("Opening the gripper is successful")
        rospy.loginfo("==================================")
        rospy.sleep(1)

# if __name__ == '__main__':
#     g = gripper()
#     g.publisher()