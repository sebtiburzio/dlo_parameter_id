#!/usr/bin/env python3

#%%
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rokubimini_msgs.srv
import tf
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def move_home(joint1=0.0,joint6=0.9):
  move_group.go(joints=[joint1, -0.1, 0.0, -1.0, 0.0, joint6, 0], wait=True) # EE close to [0.4, 0.0, 0.8] oriented Z down
  move_group.stop()


def move_to_quaternion(x,y,z, qx, qy, qz ,qw):
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.x = qx
  pose_goal.orientation.y = qy
  pose_goal.orientation.z = qz
  pose_goal.orientation.w = qw
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z
  move_group.set_pose_target(pose_goal)
  move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()


#TODO - figure out why cant plan the exectute separaetly
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
  move_group.plan()

def move_to_extr_XZY(x,y,z, X, Z, Y):
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
  move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()

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
  move_group.plan()

def move_to(x,y,z, a1, a2, a3, ax1='x', ax2='z', ax3='y', r='s'):
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
  move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()

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

def move_to_cart(x,y,z, a1, a2, a3, ax1='x', ax2='z', ax3='y', r='s'):
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
  plan = move_group.retime_trajectory(move_group.get_current_state(),plan,0.2,0.2)
  if fraction == 1.0:
    move_group.execute(plan, wait=True)
  else:
    print("ERROR: planning failed at %d of trajectory", fraction)
 

def plan_cart_path(xs,zs,phis,execute=False):
  waypoints = []

  for n in range(len(xs)):
    pose_goal = geometry_msgs.msg.Pose()
    quat = tf.transformations.quaternion_from_euler(pi, pi/4, phis[n], 'sxzy')
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = xs[n]
    pose_goal.position.y = 0.0
    pose_goal.position.z = zs[n]
    print(pose_goal)
    print(waypoints)
    waypoints.append(copy.deepcopy(pose_goal))

  (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0) # jump_threshold - TODO check if should change
  plan = move_group.retime_trajectory(move_group.get_current_state(),plan,0.03,0.03)
  if fraction == 1.0:
    print("planning success")
    if execute:
      move_group.execute(plan, wait=True)
  else:
    print("ERROR: planning failed at %d of trajectory", fraction)


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

#%%

rospy.init_node('fr3_control', anonymous=True)

print("============ Setting up moveit commander interface")
# MoveIt Commander interface init
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "fr3_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_acceleration_scaling_factor(0.1)
move_group.set_max_velocity_scaling_factor(0.1)
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
path = np.loadtxt('./static_solver_path.csv', dtype=np.float64, delimiter=',')
xs = path[:,0]
zs = path[:,1]
phis = path[:,2]
plan_cart_path(xs,zs,phis,execute=False)