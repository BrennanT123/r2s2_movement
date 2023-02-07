#!/usr/bin/env python3


import rospy
import moveit_ompl_control
from moveit_ompl_control import moveManipulator

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO 

from tf.transformations import euler_from_quaternion, quaternion_from_euler




def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class r2s2arm(object):
  """ThrowingArm Class"""
  def __init__(self):
    super(r2s2arm, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('node_r2s2arm', anonymous=True)

    # Setup Variables needed for Moveit_Commander
    self.glider_name = ''
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = "mh5l"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.planning_frame = self.move_group.get_planning_frame()
    self.eef_link = self.move_group.get_end_effector_link()
    self.group_names = self.robot.get_group_names()

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
  def go_to_pos(self):

    joint_goal = self.move_group.get_current_joint_values()

    # Define "Airplane Pickup" Position
    joint_goal[0] = -1.41
    joint_goal[1] = 1.08
    joint_goal[2] = 0.21
    joint_goal[3] = 0
    joint_goal[4] = -0.70
    joint_goal[5] = 0

    # Send action to move-to defined position
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

def main():
    try:
        robot = r2s2arm()
        robot.set_accel(0.2)
        robot.set_velo(0.2)
        input('go to defined position <enter>')
        robot.go_to_pos()
        
      except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
