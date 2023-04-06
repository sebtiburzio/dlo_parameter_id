#!/usr/bin/env python3

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  
    ## This interface can be used to plan and execute motions:
    group_name = "fr3_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    move_group.set_max_acceleration_scaling_factor(0.1)
    move_group.set_max_velocity_scaling_factor(0.1)

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -0.7156
    joint_goal[2] = 0
    joint_goal[3] = -1.693
    joint_goal[4] = 0
    joint_goal[5] = 0.9774
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def plan_pose_goal(self):

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.0
    pose_goal.position.x = 0.3
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.75

    self.move_group.set_pose_target(pose_goal)
    plan = self.move_group.plan()
    return plan


  def execute_pose(self, plan):

    self.move_group.execute(plan, wait=True)
   
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    return


  def plan_cartesian_path(self, scale=1):
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. 
    ##
    waypoints = []

    wpose = self.move_group.get_current_pose().pose

    # 3 axes
    # wpose.position.x -= scale * 0.1 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x += scale * 0.2 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x -= scale * 0.1 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.1 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y += scale * 0.2 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.1 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.z -= scale * 0.1 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.z += scale * 0.2 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.z -= scale * 0.1 
    # waypoints.append(copy.deepcopy(wpose))

    # X-axis back and forward
    wpose.position.x -= scale * 0.1 
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.2 
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= scale * 0.2 
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.2 
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= scale * 0.2 
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.2 
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= scale * 0.2 
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1 
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    self.move_group.set_max_acceleration_scaling_factor(0.5)
    self.move_group.set_max_velocity_scaling_factor(0.5)
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def plan_orientation_path(self, scale=1):
    waypoints = []

    wpose = self.move_group.get_current_pose().pose
    wpose.orientation.w = 0.0
    wpose.orientation.x = 0.966
    wpose.orientation.y = 0.0
    wpose.orientation.z = 0.259 
    waypoints.append(copy.deepcopy(wpose))

    wpose = self.move_group.get_current_pose().pose
    wpose.orientation.w = 0.0
    wpose.orientation.x = 0.966
    wpose.orientation.y = 0.0
    wpose.orientation.z = -0.259
    waypoints.append(copy.deepcopy(wpose))

    wpose = self.move_group.get_current_pose().pose
    wpose.orientation.w = 0.0
    wpose.orientation.x = 0.966
    wpose.orientation.y = 0.0
    wpose.orientation.z = 0.259 
    waypoints.append(copy.deepcopy(wpose))

    wpose = self.move_group.get_current_pose().pose
    wpose.orientation.w = 0.0
    wpose.orientation.x = 0.966
    wpose.orientation.y = 0.0
    wpose.orientation.z = -0.259
    waypoints.append(copy.deepcopy(wpose))

    wpose = self.move_group.get_current_pose().pose
    wpose.orientation.w = 0.0
    wpose.orientation.x = 1.0
    wpose.orientation.y = 0.0
    wpose.orientation.z = 0.0 
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def execute_plan(self, plan):
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    self.move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


def main():
  try:
    print("")
    print("--------------------------------")
    print("FR3 Test")
    print("--------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print("============ Press `Enter` to execute a movement using a joint state goal ...")
    input()
    tutorial.go_to_joint_state()

    #print("============ Press `Enter` to plan a movement using a pose goal ...")
    #input()
    #pose_plan = tutorial.plan_pose_goal()

    #print("============ Press `Enter` to execute a movement using a pose goal ...")
    #input()
    #tutorial.execute_pose(pose_plan)

    # print("============ Press `Enter` to plan and display a Cartesian path ...")
    # input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print("============ Press `Enter` to plan and display an orientation path ...")
    input()
    cartesian_plan, fraction = tutorial.plan_orientation_path()

    print("============ Press `Enter` to execute a saved path ...")
    input()
    tutorial.execute_plan(cartesian_plan)

    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
