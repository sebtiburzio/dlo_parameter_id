#!/usr/bin/env python3

import sys
import os
import csv
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from franka_msgs.msg import FrankaState
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from math import pi


def move_home(joint1=0.0,joint6=1.1):
  if joint1 > 0.0:
    joint1 = 0.0
    print("Clamping joint1 below 0.0 rad")
  elif joint1 < -2.5:
    joint1 = -2.5
    print("Clamping joint1 above -2.5 rad")
  move_group.go(joints=[joint1, -0.8, 0.0, -1.9, 0.0, joint6, -pi/2], wait=True)
  move_group.stop()

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
  (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
  plan = move_group.retime_trajectory(move_group.get_current_state(),plan,0.1,0.1)
  if fraction == 1.0:
    return plan
  else:
    print("ERROR: planning failed at %d of trajectory", fraction)

def execute_plan(plan):
  move_group.execute(plan, wait=True)
  move_group.stop()
  move_group.clear_pose_targets()
  plan = None

def write_csv():
  # Write data to csv
  with open('./sequence_results.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['ts', 'X_EE', 'Y_EE', 'Z_EE', 'Phi', 'Goal_X', 'Goal_Z', 'Endpt_Sol_X', 'Endpt_Sol_Z'])
    for n in range(len(ts)):
      writer.writerow([ts[n], X_EE[n], Y_EE[n], Z_EE[n], Phi_EE[n], Goal_X[n], Goal_Z[n], Endpt_Sol_X[n], Endpt_Sol_Z[n]])


if __name__ == '__main__':
    try:

        rospy.init_node('static_data_collector', anonymous=True)

        # Check required topics are available
        print("Waiting for camera")
        rospy.wait_for_message("/camera/color/image_raw/compressed", CompressedImage, timeout=None)
        print("Waiting for robot")
        rospy.wait_for_message("/franka_state_controller/franka_states", FrankaState, timeout=None)

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
        
        tflistener = tf.TransformListener()
        bridge = CvBridge()

        # Other setup
        object_Y_plane = -0.2 # Y coordinate of object plane, parallel to XZ plane
        
        # Load EE pt sequence
        sequence = np.loadtxt('./sequence.csv', dtype=np.float64, delimiter=',')
        X_seq = sequence[:,0]
        Z_seq = sequence[:,1]
        Phi_seq = sequence[:,2]
        Goals_X = sequence[:,3]
        Goals_Z = sequence[:,4]
        Endpt_Sols_X = sequence[:,5]
        Endpt_Sols_Z = sequence[:,6]

        # Data to save
        ts = []
        X_EE = []
        Y_EE = []
        Z_EE = []
        Phi_EE = []
        Goal_X = []
        Goal_Z = []
        Endpt_Sol_X = []
        Endpt_Sol_Z = []
        os.makedirs('./images')

        for i in range(len(X_seq)):
            
            # Move to better position for planning to highly angled poses
            if X_seq[i] < 0:
              move_home(joint1=-2)
            else:
              move_home(joint1=-1.15)

            print("Moving to point %d: (%3f, %3f, %3f)" % (i, X_seq[i], Z_seq[i], Phi_seq[i]))
            plan = plan_to_cart(X_seq[i], object_Y_plane, Z_seq[i], pi, 0, Phi_seq[i])
            execute_plan(plan)
            print("Ready to record data")
            input()

            # Read from topics
            img_msg = rospy.wait_for_message("/camera/color/image_raw/compressed", CompressedImage, timeout=None)
            ts.append(img_msg.header.stamp) # Use image timestamp for all data
            fr3_msg = rospy.wait_for_message("/franka_state_controller/franka_states", FrankaState, timeout=None)
            # Save image
            imgbgr = bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            cv2.imwrite('./images/' + str(ts[i]) + '.jpg', imgbgr)
            # Save data. 
            (trans,rot) = tflistener.lookupTransform('/fr3_link0', '/fr3_virtual_EE_link', rospy.Time(0))
            angs = tf.transformations.euler_from_quaternion(rot, axes='sxzy')
            X_EE.append(trans[0])
            Y_EE.append(trans[1])
            Z_EE.append(trans[2])
            Phi_EE.append(angs[2])
            # The goals and endpoints from model are passed through to make easier to match up the measurements to the input
            Goal_X.append(Goals_X[i])
            Goal_Z.append(Goals_Z[i])
            Endpt_Sol_X.append(Endpt_Sols_X[i])
            Endpt_Sol_Z.append(Endpt_Sols_Z[i])
            
            print("Data saved, ready to straighten")
            input()

        print("Sequence complete")
        move_home(joint1=-pi/2)

        write_csv()

    except EOFError:
      write_csv()
      exit()
    except rospy.ROSInterruptException:
      write_csv()
      exit()
    except KeyboardInterrupt:
      write_csv()
      exit()