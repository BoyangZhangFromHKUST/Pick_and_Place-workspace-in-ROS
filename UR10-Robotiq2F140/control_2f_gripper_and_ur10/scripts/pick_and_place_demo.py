print("ORISYS") # symbolization
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from gripper_basic_functions_ import gripper
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class Demo(object):
    def __init__(self):
        super(Demo, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    
    def move2_initial_pose(self):
        self.move_group.set_named_target("home")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -math.pi / 2
        joint_goal[2] = -math.pi / 2
        joint_goal[3] = -math.pi / 2
        joint_goal[4] = math.pi / 2
        joint_goal[5] = math.pi / 2
        
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
        current_joint_values = self.move_group.get_current_joint_values()
        current_pose = self.move_group.get_current_pose()

        print("robot joint status is: ", current_joint_values)
        print("robot pose is: ", current_pose)
        rospy.loginfo("========================================")
        
        return current_joint_values
    
    def gripper_activate(self):
        self.g = gripper()
        # rospy.init_node("control_2f_gripper")
        rospy.loginfo("Gripper Initialization")

        self.pub = rospy.Publisher(
            "Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10
        )

        gripper_ready = self.g.gripper_ready()
        self.pub.publish(gripper_ready)

        gripper_activate = self.g.gripper_activate()
        self.pub.publish(gripper_activate)

        rospy.sleep(.5)

        gripper_open = self.g.gripper_open()
        self.pub.publish(gripper_open)

        rospy.sleep(1)
        rospy.loginfo("Initialization is successful!")
        rospy.loginfo("========================================")

    def move2_object_pose(self):
        # self.move_group.set_named_target("object")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = math.radians(40)
        joint_goal[1] = math.radians(-110)
        joint_goal[2] = math.radians(-107)
        joint_goal[3] = math.radians(-52)
        joint_goal[4] = math.radians(90)
        joint_goal[5] = math.radians(130)
        
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
        current_joint_values = self.move_group.get_current_joint_values()
        current_pose = self.move_group.get_current_pose()

        print("robot joint status is: ", current_joint_values)
        print("robot pose is: ", current_pose)
        rospy.loginfo("========================================")

        return current_joint_values
    
    def gripper_close(self):
        up = self.g.up_threshold()
        down = self.g.down_threshold()

        for i in range(down,up,10):
            gripper_grasp = self.g.gripper_grasp(i)
            self.pub.publish(gripper_grasp)
            # rospy.sleep(0.1) # the frequency of grasping
        
        rospy.loginfo("========================================")
        rospy.sleep(1)

    def move2_final_pose(self):
        # self.move_group.set_named_target("final")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = math.radians(50)
        joint_goal[1] = math.radians(-78)
        joint_goal[2] = math.radians(-106)
        joint_goal[3] = math.radians(-60)
        joint_goal[4] = math.radians(110)
        joint_goal[5] = math.radians(140)
        
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
        current_joint_values = self.move_group.get_current_joint_values()
        current_pose = self.move_group.get_current_pose()

        print("robot joint status is: ", current_joint_values)
        print("robot pose is: ", current_pose)
        rospy.loginfo("========================================")

        return current_joint_values
    
    def gripper_open(self):
        gripper_open = self.g.gripper_open()
        self.pub.publish(gripper_open)
        rospy.sleep(0.1)
        rospy.loginfo("========================================")
        rospy.sleep(1)

    def plan_cartesian_path(self):
        move_group = self.move_group
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z += 0.1
        wpose.position.y += 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x -= 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        
        return plan, fraction
    
    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)
        move_group.stop()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": I heard %s", data.data)

def ros_listener():

    subscriber = rospy.Subscriber("touch_signal", String, callback)

    while subscriber.get_num_connections() == 0:
        rospy.loginfo("Waiting for touching")
        rospy.sleep(0.1)
    print(subscriber)

def main():
    try:
        demo = Demo()
        
        rospy.sleep(1)
        
        demo.move2_initial_pose()
        rospy.sleep(.5)

        demo.gripper_activate()
        rospy.sleep(3)

        demo.move2_object_pose()
        rospy.sleep(.5)

        demo.gripper_close()
        rospy.sleep(.5)

        ros_listener()

        demo.move2_final_pose()
        rospy.sleep(.5)

        demo.gripper_open()
        rospy.sleep(.5)

        demo.move2_initial_pose()
        rospy.sleep(.5)

        # demo.go_to_pose()
        cartesian_plan, fraction = demo.plan_cartesian_path()

        demo.execute_plan(cartesian_plan, fraction)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()