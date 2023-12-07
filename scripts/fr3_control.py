#!/usr/bin/env python3

#%%
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np
from math import pi


def move_home(joint1=0.0,joint6=1.1):
  if joint1 > 0.0:
    joint1 = 0.0
    print("Clamping joint1 below 0.0 rad")
  elif joint1 < -2.5:
    joint1 = -2.5
    print("Clamping joint1 above -2.5 rad")
  move_group.go(joints=[joint1, -0.55, 0.0, -1.65, 0.0, joint6, -pi/2], wait=True)
  move_group.stop()

def plan_to_quaternion(x,y,z, qx, qy, qz ,qw):
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.x = qx
  pose_goal.orientation.y = qy
  pose_goal.orientation.z = qz
  pose_goal.orientation.w = qw
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z
  move_group.set_pose_target(pose_goal)
  plan = move_group.plan()
  return plan

def plan_to_extr_XZY(x,y,z, X, Z, Y):
  pose_goal = geometry_msgs.msg.Pose()
  quat = tf.transformations.quaternion_from_euler(X, Z, Y, 'sxzy')
  pose_goal.orientation.x = quat[0]
  pose_goal.orientation.y = quat[1]
  pose_goal.orientation.z = quat[2]
  pose_goal.orientation.w = quat[3]
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z
  move_group.set_pose_target(pose_goal)
  plan = move_group.plan()
  return plan

def plan_to(x,y,z, a1, a2, a3, ax1='x', ax2='z', ax3='y', r='s'):
  pose_goal = geometry_msgs.msg.Pose()
  quat = tf.transformations.quaternion_from_euler(a1, a2, a3, r + ax1 + ax2 + ax3)
  pose_goal.orientation.x = quat[0]
  pose_goal.orientation.y = quat[1]
  pose_goal.orientation.z = quat[2]
  pose_goal.orientation.w = quat[3]
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z
  move_group.set_pose_target(pose_goal)
  plan = move_group.plan()
  return plan

def plan_to_cart(x,y,z, a1, a2, a3, ax1='x', ax2='z', ax3='y', r='s'):
  waypoints = []
  pose_goal = geometry_msgs.msg.Pose()
  quat = tf.transformations.quaternion_from_euler(a1, a2, a3, r + ax1 + ax2 + ax3)
  pose_goal.orientation.x = quat[0]
  pose_goal.orientation.y = quat[1]
  pose_goal.orientation.z = quat[2]
  pose_goal.orientation.w = quat[3]
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z
  waypoints.append(copy.deepcopy(pose_goal))
  (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0) # jump_threshold - TODO check if should change
  plan = move_group.retime_trajectory(move_group.get_current_state(),plan,0.05,0.05)
  if fraction == 1.0:
    return plan
  else:
    print("ERROR: planning failed at %d of trajectory", fraction)

def plan_cart_path(xs,zs,phis):
  waypoints = []

  for n in range(len(xs)):
    pose_goal = geometry_msgs.msg.Pose()
    quat = tf.transformations.quaternion_from_euler(pi, 0, phis[n], 'sxzy')
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = xs[n]
    pose_goal.position.y = -0.2
    pose_goal.position.z = zs[n]
    print(pose_goal)
    print(waypoints)
    waypoints.append(copy.deepcopy(pose_goal))

  (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0) # jump_threshold - TODO check if should change
  plan = move_group.retime_trajectory(move_group.get_current_state(),plan,0.03,0.03)
  if fraction == 1.0:
    return plan
  else:
    print("ERROR: planning failed at %d of trajectory", fraction)

def execute_plan(plan):
    move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    plan = None


#%%

rospy.init_node('fr3_control', anonymous=True)

print("============ Setting up moveit commander interface")
# MoveIt Commander interface init
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "fr3_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_acceleration_scaling_factor(0.1) # Note these don't affect cartesian paths. Use retime_trajectory
move_group.set_max_velocity_scaling_factor(0.1)     #
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

#%%
# Path from a csv file
path = np.loadtxt('./orange_grid_horiz_RHS.csv', dtype=np.float64, delimiter=',')
xs = path[:,0]
zs = path[:,1]
phis = path[:,2]
# To first point
plan = plan_to_cart(xs[0], -0.2, zs[0], pi, 0, phis[0])
#%%
# Full path
plan = plan_cart_path(xs,zs,phis)
