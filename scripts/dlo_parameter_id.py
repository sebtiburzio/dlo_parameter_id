#!/usr/bin/env python3

import sys
import copy
import rospy
import dynamic_reconfigure.client
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rokubimini_msgs.srv
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def align_ft_z_at(x=0.3,y=0.0,z=0.55):
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.x = 1.0
  pose_goal.orientation.y = 0.0
  pose_goal.orientation.z = 0.0
  pose_goal.orientation.w = 0.0
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z
  move_group.set_pose_target(pose_goal)
  move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()


def reset_ft_gravity_aligned(mass=None):

  ft_offset = geometry_msgs.msg.Wrench()
  ft_offset.torque.x = 0.0
  ft_offset.torque.y = 0.0
  ft_offset.torque.z = 0.0
  ft_offset.force.x = 0.0
  ft_offset.force.y = 0.0

  if mass = None:
    try:
      gripper_mass = rospy.get_param("/imu_gravity_compensation/gripper_mass")
    except KeyError:
      print("WARNING: No gripper mass found for gravity compensation offset")
      gripper_mass = 0.0
  else:
    gripper_mass = mass

  ft_offset.force.z = gripper_mass*9.81

  reset_ft_wrench_srv(ft_offset)


def actuate_x_axis(distance=0.5, cycles=2):
  waypoints = []
  wpose = move_group.get_current_pose().pose
 
  wpose.position.x += distance
  waypoints.append(copy.deepcopy(wpose))
  for n in range(cycles):
    wpose.position.x -= 2*distance
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x += 2*distance
    waypoints.append(copy.deepcopy(wpose))
  wpose.position.x -= distance
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0) # jump_threshold - TODO check if should change
  if fraction == 1.0:
    move_group.execute(plan, wait=True)
  else:
    print("ERROR: planning failed at %d of trajectory", fraction)


def actuate_joint_6(angle=pi/12, cycles=2):
  curJoints = move_group.get_current_state().joint_state.position
  move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5]+angle, curJoints[6]], wait=True)
  move_group.stop()

  for n in range(cycles):
    curJoints = move_group.get_current_state().joint_state.position
    move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5]-2*angle, curJoints[6]], wait=True)
    move_group.stop()
    
    curJoints = move_group.get_current_state().joint_state.position
    move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5]+2*angle, curJoints[6]], wait=True)
    move_group.stop()

  curJoints = move_group.get_current_state().joint_state.position
  move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5]-angle, curJoints[6]], wait=True)
  move_group.stop()


def actuate_phi_axis(cycles=2):
  waypoints = []
  wpose = move_group.get_current_pose().pose

  for n in range(cycles):
    wpose.orientation.w = 0.0
    wpose.orientation.x = 0.966
    wpose.orientation.y = 0.0
    wpose.orientation.z = 0.259 
    waypoints.append(copy.deepcopy(wpose))
    wpose.orientation.w = 0.0
    wpose.orientation.x = 0.966
    wpose.orientation.y = 0.0
    wpose.orientation.z = -0.259 
    waypoints.append(copy.deepcopy(wpose))
  wpose.orientation.w = 0.0
  wpose.orientation.x = 1.0
  wpose.orientation.y = 0.0
  wpose.orientation.z = 0.0 
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0) # jump_threshold - TODO check if should change
  if fraction == 1.0:
    move_group.execute(plan, wait=True)
  else:
    print("ERROR: planning failed at %d of trajectory", fraction)


def ft_orientation_cycle():
  curJoints = move_group.get_current_state().joint_state.position
  print("============ Press enter")
  input()
  move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5], curJoints[6]+pi/2], wait=True)
  move_group.stop()

  curJoints = move_group.get_current_state().joint_state.position
  print("============ Press enter")
  input()
  move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5], curJoints[6]-pi], wait=True)
  move_group.stop()

  curJoints = move_group.get_current_state().joint_state.position
  print("============ Press enter")
  input()
  move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5], curJoints[6]+pi/2], wait=True)
  move_group.stop()


if __name__ == '__main__':
  rospy.init_node('dlo_parameter_id', anonymous=True)

  print("============ Setting up moveit commander interface")
  # MoveIt Commander interface init
  moveit_commander.roscpp_initialize(sys.argv)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group_name = "fr3_arm"
  move_group = moveit_commander.MoveGroupCommander(group_name)
  move_group.set_max_acceleration_scaling_factor(0.3)
  move_group.set_max_velocity_scaling_factor(0.3)
  planning_frame = move_group.get_planning_frame()
  print("============ Planning frame: %s" % planning_frame)
  eef_link = move_group.get_end_effector_link()
  print("============ End effector link: %s" % eef_link)
  group_names = robot.get_group_names()
  print("============ Available Planning Groups:", robot.get_group_names())
  print("============ Robot state:")
  print(robot.get_current_state())
  print("")
  # For displaying trajectory plans in RViz
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

  # Service for resetting FT sensor bias
  reset_ft_wrench_srv = rospy.ServiceProxy('/bus0/ft_sensor0/reset_wrench', rokubimini_msgs.srv.ResetWrench())

  print("============ Press enter to vertically align and zero FT sensor")
  input()
  #move_group.go(joints=[0.0, -0.7156, 0.0, -1.693, 0.0, 0.9774, 0], wait=True) # EE close to [0.2, 0.0, 0.75] oriented Z down
  move_group.go(joints=[0.0, -0.1, 0.0, -1.0, 0.0, 0.9, 0], wait=True) # EE close to [0.4, 0.0, 0.8] oriented Z down
  move_group.stop()
  curPos = move_group.get_current_pose().pose.position
  align_ft_z_at(x=0.4,y=0.0,z=0.8)
  reset_ft_gravity_aligned(0.72)
  
  # print("============ Press enter to actuate link 6")
  # input()
  # actuate_joint_6()

  print("============ Press enter to actuate x axis")
  input()
  actuate_x_axis(0.03,2)

  #print("============ Press enter to actuate phi axis")
  #input()
  #actuate_phi_axis()
  
