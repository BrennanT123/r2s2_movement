#!/usr/bin/env python3


from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


## from moveit example code
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    #tau = 2.0 * pi
    tau = 0

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
        group_name = "mh5l_arm"
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
        joint_goal[0] = .151
        joint_goal[1] = .51
        joint_goal[2] = .33
        joint_goal[3] = 2.25
        joint_goal[4] = .125
        joint_goal[5] = 1.012 

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)



    def go_to_pose_goal(self):

        move_group = self.move_group


        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0000001
        pose_goal.orientation.x = .1000001
        pose_goal.orientation.y = .1000002
        pose_goal.orientation.z = .1000001
        
        pose_goal.position.x = .8
        pose_goal.position.y = .5
        pose_goal.position.z = .2

        move_group.set_pose_target(pose_goal)

 
        success = move_group.go(wait=True)

        move_group.stop()
 
        move_group.clear_pose_targets()


        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_throw_path(self, scale=1):
        ## Plan Cartesian Path to throw glider

        # Specify a list of waypoints
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z += scale * 0.1  # Move up (z)
        wpose.position.x += scale * 0.8  # Forward (x)
        waypoints.append(copy.deepcopy(wpose))



        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.05,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction



        
def main():
    try:

        tutorial = MoveGroupRcycl()

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        tutorial.go_to_joint_state()

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal()


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

