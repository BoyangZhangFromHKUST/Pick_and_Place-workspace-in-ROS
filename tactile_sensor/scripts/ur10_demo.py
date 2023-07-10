import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String

class Demo(object):
    def __init__(self):
        super(Demo, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -math.pi / 2
        joint_goal[2] = -math.pi / 2
        joint_goal[3] = -math.pi / 2
        joint_goal[4] = math.pi / 2
        joint_goal[5] = math.pi / 2  # 1/6 of a turn
        # joint_goal[6] = 0

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joint_values = move_group.get_current_joint_values()
        
        return current_joint_values
    
    def go_to_pose(self):
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = -0.5 # meter
        pose_goal.position.y = 0.25 # meter
        pose_goal.position.z = 1.0  # meter

        move_group.set_pose_target(pose_goal)

        move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = move_group.get_current_pose().pose

        return current_pose
    
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
    
    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory_msg = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory_msg.trajectory_start = robot.get_current_state()
        display_trajectory_msg.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory_msg)

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

    # rospy.spin()
    # rospy.signal_shutdown("shutdown")



def main():
    try:
        demo = Demo()

        ros_listener()
        
        rospy.sleep(1)
        demo.go_to_joint_state()
        # demo.go_to_pose()
        cartesian_plan, fraction = demo.plan_cartesian_path()

        demo.display_trajectory(cartesian_plan)
        demo.execute_plan(cartesian_plan, fraction)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
