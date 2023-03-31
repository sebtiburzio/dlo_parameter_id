#!/usr/bin/env python

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


def reset_ft_gravity_aligned():
  try:
    gripper_mass = rospy.get_param("/imu_gravity_compensation/gripper_mass")
  except KeyError:
    print "WARNING: No gripper mass found for gravity compensation offset"
    gripper_mass = 0.0
  ft_offset = geometry_msgs.msg.Wrench()
  ft_offset.force.x = 0.0
  ft_offset.force.y = 0.0
  ft_offset.force.z = gripper_mass*9.81
  ft_offset.torque.x = 0.0
  ft_offset.torque.y = 0.0
  ft_offset.torque.z = 0.0
  reset_ft_wrench_srv(ft_offset)


def actuate_x_axis(distance=0.1, cycles=2):
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
    print "ERROR: planning failed at %d of trajectory", fraction


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
    print "ERROR: planning failed at %d of trajectory", fraction


def ft_orientation_cycle():
  move_group.go(joints=[0.0, -pi/4, 0.0, -3*pi/4, 0.0, pi/2, 0], wait=True) # EE close to [0.3, 0.0, 0.75] oriented Z down
  move_group.stop()
  reset_ft_gravity_aligned()
  print "============ Press enter"
  raw_input()
  move_group.go(joints=[0.0, -pi/4, 0.0, -3*pi/4, 0.0, pi, 0], wait=True) # EE close to [0.3, 0.0, 0.75] oriented Z down
  move_group.stop()
  print "============ Press enter"
  raw_input()
  move_group.go(joints=[0.0, -pi/4, 0.0, -3*pi/4, 0.0, pi/2, 0], wait=True) # EE close to [0.3, 0.0, 0.75] oriented Z down
  move_group.stop()


# def actuate_swing():
#     waypoints = []
#     wpose = move_group.get_current_pose().pose

#     wpose.position.x -= 0.03
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 0.991
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = 0.131
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x -= 0.02
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 1.0
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = 0.0
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x += 0.02
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 0.991
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = -0.131
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x += 0.08
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x -= 0.02
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 1.0
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = 0.0
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x -= 0.08
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 0.991
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = 0.131
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x += 0.02
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 1.0
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = 0.0
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x += 0.02
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 0.991
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = -0.131
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x += 0.08
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x -= 0.02
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 1.0
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = 0.0
#     waypoints.append(copy.deepcopy(wpose))
#     wpose.position.x -= 0.08
#     wpose.orientation.w = 0.0
#     wpose.orientation.x = 0.991
#     wpose.orientation.y = 0.0
#     wpose.orientation.z = 0.131

#     (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0) # jump_threshold - TODO check if should change
#     print "============ Press enter to actuate swing"
#     raw_input()
#     if fraction == 1.0:
#       move_group.execute(plan, wait=True)
#     else:
#       print "ERROR: planning failed at %d of trajectory", fraction


if __name__ == '__main__':
  rospy.init_node('dlo_parameter_id', anonymous=True)

  print "============ Setting up moveit commander interface"
  # MoveIt Commander interface init
  moveit_commander.roscpp_initialize(sys.argv)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group_name = "fr3_arm"
  move_group = moveit_commander.MoveGroupCommander(group_name)
  move_group.set_max_acceleration_scaling_factor(0.5)
  move_group.set_max_velocity_scaling_factor(0.5)
  planning_frame = move_group.get_planning_frame()
  print "============ Planning frame: %s" % planning_frame
  eef_link = move_group.get_end_effector_link()
  print "============ End effector link: %s" % eef_link
  group_names = robot.get_group_names()
  print "============ Available Planning Groups:", robot.get_group_names()
  print "============ Robot state:"
  print robot.get_current_state()
  print ""
  # For displaying trajectory plans in RViz
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

  # Service for resetting FT sensor bias
  reset_ft_wrench_srv = rospy.ServiceProxy('/bus0/ft_sensor0/reset_wrench', rokubimini_msgs.srv.ResetWrench())

  print "============ Press enter to vertically align and zero FT sensor"
  raw_input()
  move_group.go(joints=[0.0, -pi/4, 0.0, -3*pi/4, 0.0, pi/2, 0], wait=True) # EE close to [0.3, 0.0, 0.75] oriented Z down
  move_group.stop()
  curPos = move_group.get_current_pose().pose.position
  align_ft_z_at(x=curPos.x,y=curPos.y,z=curPos.z)
  reset_ft_gravity_aligned()

  # curJoints = move_group.get_current_state().joint_state.position
  # print "============ Press enter"
  # raw_input()
  # move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5]+pi/2, curJoints[6]], wait=True)
  # move_group.stop()

  curJoints = move_group.get_current_state().joint_state.position
  print "============ Press enter"
  raw_input()
  move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5], curJoints[6]+pi/2], wait=True)
  move_group.stop()

  curJoints = move_group.get_current_state().joint_state.position
  print "============ Press enter"
  raw_input()
  move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5], curJoints[6]-pi], wait=True)
  move_group.stop()

  curJoints = move_group.get_current_state().joint_state.position
  print "============ Press enter"
  raw_input()
  move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5], curJoints[6]+pi/2], wait=True)
  move_group.stop()

  # curJoints = move_group.get_current_state().joint_state.position
  # print "============ Press enter"
  # raw_input()
  # move_group.go(joints=[curJoints[0], curJoints[1], curJoints[2], curJoints[3], curJoints[4], curJoints[5]-pi/2, curJoints[6]], wait=True)
  # move_group.stop()

  # print "============ Press enter to actuate x axis"
  # raw_input()
  # actuate_x_axis(0.03, 1)

  # print "============ Press enter to actuate phi axis"
  # raw_input()
  # actuate_phi_axis()
