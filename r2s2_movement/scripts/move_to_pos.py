#!/usr/bin/env python3


from __future__ import print_function
from six.moves import input


#import message_generation 
import std_msgs
from sensor_msgs.msg import Image, CameraInfo

import re
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from motoman_msgs.srv import ReadSingleIO, WriteSingleIO



## from moveit example code
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))




def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


def act_gripper(request):
    ## Wrapper for rosservice to open/close gripper using Read/Write IO

    # Wait for ros services to come up
    rospy.wait_for_service('read_single_io')
    rospy.wait_for_service('write_single_io')

    # Create Handle for Service Proxy's
    try:
        read_single_io = rospy.ServiceProxy('read_single_io', ReadSingleIO)
        write_single_io = rospy.ServiceProxy('write_single_io', WriteSingleIO)
    except rospy.ServiceException as e:
        print("Gripper IO Service Call failed: %s" % e)

    # Send 'Write' IO Message
    try:
        write_status = write_single_io(10010, request)
    except:
        print("An exception occured. Unable to write to Single IO.")

    # Call Read Service to check current position
    read_status = read_single_io(10011).value
    if read_status:
        print('Gripper is Closed')
    else:
        print('Gripper is Open')

    return read_status


class MoveGroupRcycl(object):
    """MoveGroupRcycl"""

    def __init__(self):
        super(MoveGroupRcycl, self).__init__()


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
        #group_name = "mh5l_pgn64"
        group_name = "mh5l"
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



    def go_to_joint_state(self, goals):
      
        move_group = self.move_group
        print(goals)
    
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = goals[0]
        joint_goal[1] = goals[1]
        joint_goal[2] = goals[2]
        joint_goal[3] = goals[3]
        joint_goal[4] = goals[4]
        joint_goal[5] = goals[5]  

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)



    def go_to_pose_goal(self):
        pose = [.01, 0.1, 0.01, 0.1, .2, 0.1]

        #move_group = self.move_group
        q_goal = quaternion_from_euler(pose[3],pose[4],pose[5],axes='sxyz')

        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.w = q_goal[0]
        pose_goal.orientation.x = q_goal[1]
        pose_goal.orientation.y = q_goal[2]
        pose_goal.orientation.z = q_goal[3]


        self.move_group.set_pose_target(pose_goal)

        #execute planning and go to position
        plan = self.move_group.go(wait=True)
        #stops so no residual movement
        plan = self.move_group.stop()

        self.move_group.clear_pose_targets()


        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def act_gripper(request):
    ## Wrapper for rosservice to open/close gripper using Read/Write IO

    # Wait for ros services to come up
        rospy.wait_for_service('read_single_io')
        rospy.wait_for_service('write_single_io')

    # Create Handle for Service Proxy's
        try:
            read_single_io = rospy.ServiceProxy('read_single_io', ReadSingleIO)
            write_single_io = rospy.ServiceProxy('write_single_io', WriteSingleIO)
        except rospy.ServiceException as e:
            print("Gripper IO Service Call failed: %s" % e)

    # Send 'Write' IO Message
        try:
            write_status = write_single_io(10010, request)
        except:
            print("An exception occured. Unable to write to Single IO.")

    # Call Read Service to check current position
        read_status = read_single_io(10011).value
        if read_status:
            print('Gripper is Closed')
        else:
            print('Gripper is Open')

        return read_status
     #/robot_enable
    def plan_cartesian_path(self, scale=1):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)

        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
    def execute_plan(self, plan):
        ## Execute a Plan
        ## Use execute if you would like the robot to follow a plan that has already been computed:
        self.move_group.execute(plan, wait=True)
        return
    def set_vel(self,max_vel):
        ## Wrapper for Moveit Commander's max_velocity
        ## Allowed range...   0.0 <= max_vel <= 1.0
        self.move_group.set_max_velocity_scaling_factor(max_vel)
        
    def set_accel(self,max_accel):
        ## Wrapper for Moveit Commander's max_acceleration
        ## Allowed range...   0.0 <= max_vel <= 1.0
        self.move_group.set_max_acceleration_scaling_factor(max_accel)

    def goto_all_zeros(self):
        ## Go to "ALL-Zeros" position
        ## Get Current Position & Go to "All-Zeros" Position
        ## Trajectory Type: JOINT MOTION defined by joint position

        # Get Current Position
        joint_goal = self.move_group.get_current_joint_values()

        # Define "All-Zeros" Position
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0

        # Send action to move-to defined position
        self.move_group.go(joint_goal, wait=True)


        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()


        return all_close(joint_goal, current_joints, 0.01)
    def plan_cart_path(self, scale=1):
        ## Plan Cartesian Path

        # Specify a list of waypoints
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z += scale * -0.1  # Move up (z)
        wpose.position.x += scale * 0
        wpose.position.y += scale* 0  # Forward (x)
        waypoints.append(copy.deepcopy(wpose))



        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


        
def main():
    try:
        robot = MoveGroupRcycl()

        #predefine message and pull below

        #rospy.init_node('movement_node')
        #sub_topic_info = "camera/color/neural_network"
        robot.set_accel(0.1)
        robot.set_vel(0.1)
        #robot.go_to_joint_state([0,0,0,0,0,0])
        input("=========Return to home=========")
        robot.goto_all_zeros()
        

        input("===turn eef down ===")
        #robot.go_to_joint_state([0,0,0,0,-pi/2,0])
        #input("=======Get Camera Data==========")
        #info_sub = rospy.wait_for_message(sub_topic_info, CameraInfo)
        input("=======Execute========")
        robot.go_to_pose_goal()
        #[robot_plan, fraction] = robot.plan_cartesian_path()

        input('===executing plan=')

        #executing plan
        #robot.move_group.execute(robot_plan, wait=True)
        #robot.execute_plan(robot_plan)

        input("======griper=======")
        #rospy.init_node('node_gripper', anonymous=True)
        act_gripper(1)
        




        

        #go  to home state
        #wait for rospy message
        #go to cartesian points
        #go to bin
        #loop
 


        #coord = [0, 0, 0, 0, 0, 0]
        #info_sub = rospy.wait_for_message(sub_topic_info, CameraInfo)

        #print("Going to move to...")
        #print(info_sub)

        #input("====== Press enter if this is correct ==========")

        #input("============ Press `Enter` to execute a movement using a joint state goal ...")
        #tutorial.go_to_joint_state(coord)
        #print(coord)
        #input("====")
        #tutorial.go_to_joint_state([0, 0, 0, 0, 0, 0])
        #input("============ Press `Enter` to execute a movement using a pose goal ...")
        #tutorial.go_to_pose_goal()


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

