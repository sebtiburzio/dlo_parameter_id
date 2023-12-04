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

from generated_functions.floating.floating_base_functions import eval_fk, eval_J_end_wrt_base
from generated_functions.fixed.fixed_base_functions import eval_midpt, eval_endpt, eval_J_midpt, eval_J_endpt
target_evaluators = [eval_midpt, eval_endpt, eval_J_midpt, eval_J_endpt]
from utils import rot_XZ_on_Y, get_FK, find_curvature

def move_home(joint1=0.0,joint6=1.1):
  if joint1 > 0.0:
    joint1 = 0.0
    print("Clamping joint1 below 0.0 rad")
  elif joint1 < -2.5:
    joint1 = -2.5
    print("Clamping joint1 above -2.5 rad")
  move_group.go(joints=[joint1, -0.8, 0.0, -1.9, 0.0, joint6, -pi/2], wait=True)
  move_group.stop()

# TODO - plan then execute separately only seems to work for plan_to_cart
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

# Pixel to 3D conversions
def UV_to_XZplane(u,v,Y=0):
  rhs1 = np.hstack([P[:,:3],np.array([[-u,-v,-1]]).T])
  rhs1 = np.vstack([rhs1, np.array([0,1,0,0])])   # Intersect y=Y plane
  rhs2 = np.reshape(np.hstack([-P[:,3],[Y]]),(4,1))
  sol = np.linalg.inv(rhs1)@rhs2
  return sol[:3]

def write_csv():
  # Write data to csv
  with open('./sequence_results.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['ts', 'X_EE', 'Z_EE', 'Phi', 
                      'X_base_meas', 'Z_base_meas', 
                      'X_mid_meas', 'Z_mid_meas', 
                      'X_end_meas', 'Z_end_meas', 
                      'U_base', 'V_base', 
                      'U_mid', 'V_mid', 
                      'U_end', 'V_end', 
                      'Base_angle', 
                      'X_ang_start', 'Z_ang_start', 
                      'X_ang_end', 'Z_ang_end', 
                      'U_ang_start', 'V_ang_start', 
                      'U_ang_end', 'V_ang_end'])
    for n in range(len(ts)):
      writer.writerow([ts[n], X_EE[n], Z_EE[n], Phi_EE[n], 
                        X_base_meas[n], Z_base_meas[n], 
                        X_mid_meas[n], Z_mid_meas[n], 
                        X_end_meas[n], Z_end_meas[n],
                        U_base[n], V_base[n], 
                        U_mid[n], V_mid[n], 
                        U_end[n], V_end[n],
                        Base_angle[n], 
                        X_ang_start[n], Z_ang_start[n], 
                        X_ang_end[n], Z_ang_end[n],
                        U_ang_start[n], V_ang_start[n], 
                        U_ang_end[n], V_ang_end[n]])


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

        # Camera intrinsic and extrinsic transforms
        with np.load('../TFs_adj.npz') as tfs:
            P = tfs['P']
            E_base = tfs['E_base']
            E_cam = tfs['E_cam']
            K_cam = tfs['K_cam']

        # Other setup
        with np.load('/home/mossy/Documents/Delft/MSc_students/Seb/dlo_parameter_id/object_parameters/black_weighted.npz') as obj_params:
          p_vals = list(obj_params['p_vals']) # cable properties: mass (length), mass (end), length, diameter
        object_Y_plane = -0.2 # Y coordinate of object plane, parallel to XZ plane
        base_Y = object_Y_plane - 0.01
        mid_Y = object_Y_plane - 0.01
        end_Y = object_Y_plane - 0.015
        print("Assuming base at Y=" + str(base_Y))
        print("Assuming mid at Y=" + str(mid_Y))
        print("Assuming end at Y=" + str(end_Y) + '\n')
        
        # Load EE pt sequence
        sequence = np.loadtxt('./sequence.csv', dtype=np.float64, delimiter=',')
        X_seq = sequence[:,0]
        Z_seq = sequence[:,1]
        Phi_seq = sequence[:,2]
        Theta0 = sequence[:,3]
        Theta1 = sequence[:,4]
        GoalX = sequence[:,5]
        GoalZ = sequence[:,6]
        Endpt_X = sequence[:, 7]
        Endpt_Z = sequence[:, 8]

        # Data to save
        ts = []
        X_EE = []
        Z_EE = []
        Phi_EE = []
        X_base_meas = []
        Z_base_meas = []
        X_mid_meas = []
        Z_mid_meas = []
        X_end_meas = []
        Z_end_meas = []
        U_base = []
        V_base = []
        U_mid = []
        V_mid = []
        U_end = []
        V_end = []
        U_ang_start = []
        V_ang_start = []
        U_ang_end = []
        V_ang_end = []
        X_ang_start = []
        Z_ang_start = []
        X_ang_end = []
        Z_ang_end = []
        Base_angle = []
        # TODO need to check if this is already created
        os.makedirs('./images')

        for i in range(len(X_seq)):
            
            # Move to better position for planning to highly angled poses
            # if X_seq[i] < 0:
            #   move_home(joint1=-2)
            # else:
            #   move_home(joint1=-1.15)

            print("Planning to point %d: (%3f, %3f, %3f)" % (i, X_seq[i], Z_seq[i], Phi_seq[i]))
            plan = plan_to_cart(X_seq[i], object_Y_plane, Z_seq[i], pi, 0, Phi_seq[i])
            print("Ready to move to point")
            input()
            # execute_plan(plan)
            print("Ready to record data")
            input()

            # Read from topics
            img_msg = rospy.wait_for_message("/camera/color/image_raw/compressed", CompressedImage, timeout=None)
            ts.append(img_msg.header.stamp) # Use image timestamp for all data
            fr3_msg = rospy.wait_for_message("/franka_state_controller/franka_states", FrankaState, timeout=None)
            # Process image
            imgbgr = bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            img = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB)
            fig, ax = plt.subplots(figsize=(16, 12))
            ax.imshow(img)
            plt.axis('off')
            print("Select marker points (base, mid, end) in the image; left click - add, right - remove, middle - finish")
            # Select 5 points on the image, first 3 for the curvature, last 2 for the tip orientation
            UVs = plt.ginput(n=-1, timeout=0)
            plt.close()
            base_UV = [int(UVs[0][0]), int(UVs[0][1])]
            mid_UV = [int(UVs[1][0]), int(UVs[1][1])]
            end_UV = [int(UVs[2][0]), int(UVs[2][1])]
            base_XZ = UV_to_XZplane(base_UV[0], base_UV[1], base_Y)
            mid_XZ = UV_to_XZplane(mid_UV[0], mid_UV[1], mid_Y)
            end_XZ = UV_to_XZplane(end_UV[0], end_UV[1], end_Y)
            if len(UVs) > 3:
              base_ang_start_UV = [int(UVs[3][0]), int(UVs[3][1])] # TODO - change to alpha_ang
              base_ang_end_UV = [int(UVs[4][0]), int(UVs[4][1])]
              base_ang_start_XZ = UV_to_XZplane(base_ang_start_UV[0], base_ang_start_UV[1], end_Y)
              base_ang_end_XZ = UV_to_XZplane(base_ang_end_UV[0], base_ang_end_UV[1], end_Y)
              base_ang = np.arctan2(-(base_ang_end_XZ[0]-base_ang_start_XZ[0]),-(base_ang_end_XZ[2]-base_ang_start_XZ[2])) # atan2(-delX,-delZ) because of robot axis shenanigans
            else:
              base_ang_start_UV = [0,0]
              base_ang_end_UV = [0,0]
              base_ang_start_XZ = [0.0,0.0,0.0]
              base_ang_end_XZ = [0.0,0.0,0.0]
              base_ang = 0.0
            # TODO this fails if the .images folder is already there
            cv2.imwrite('./images/' + str(ts[i]) + '.jpg', imgbgr)

            # Process robot state/ end effector position data
            # trans and rot of virtual_EE link i.e. the red point/tape on cable
            (trans, EE_virtual_rot) = tflistener.lookupTransform('/fr3_link0', '/fr3_virtual_EE_link', rospy.Time(0))
            EE_virtual_angs = tf.transformations.euler_from_quaternion(EE_virtual_rot, axes='sxzy')

            #   ### IN FEEDBACK LOOP
            #---- magic numbers ----#
            k_gain = 1.0 # proportional gain to the error 
            error_tol = 0.001 

            # ---- initial esitimate of predicted and measured tip pose ---- #
            # TODO need to format these to get the norm between them
            predicted_tip_pose_initial= np.array([Endpt_X[0], Endpt_Z[0]]) # Endpt_X, Endpt_Z are from a generated .csv file
            print("end_XZ: ", end_XZ)
            measured_tip_pose_initial = np.array([end_XZ[0][0], end_XZ[2][0]]) # [x, y, z] want to use indexes to get x and z value 
            print("predicted_tip_pose_initial: ", predicted_tip_pose_initial)
            print("measured_tip_pose_initial: ", measured_tip_pose_initial)
            # Initial measurement of tip pose from camera 
            # TODO theta_guess - from .csv

            # theta_extracted, convergence = find_curvature(p_vals, theta_guess, target_evaluators, fk_targets, 0.005)

            print("error value: ", np.linalg.norm(predicted_tip_pose_initial - measured_tip_pose_initial))
            if np.linalg.norm(predicted_tip_pose_initial - measured_tip_pose_initial) > error_tol:
            #   # Extract curvature from marker points
            #   # Transform marker points to fixed PAC frame (subtract X/Z, rotate back phi)
            # fk_targets - moves reference from of curves measured from camera from robot frame to object frame (~EE frame)
                
              
              print("mid_XZ: ", mid_XZ)
              print("angs: ", EE_virtual_angs)
              print("end_XZ: ", mid_XZ)
              # TODO check if this needs a translation along with the rotation. take base_XZ away to move it to object frame (~EE frame)
              mid_X = mid_XZ[0][0]
              mid_Z = mid_XZ[2][0]

              end_X = end_XZ[0][0]
              end_Z = end_XZ[2][0]

              base_X = base_XZ[0][0]
              base_Z = base_XZ[2][0]

              # fk_targets = np.hstack([rot_XZ_on_Y(np.array([mid_XZ[0, 0],mid_XZ[0,2]]),-EE_virtual_angs[2]), rot_XZ_on_Y(np.array([end_XZ[0,0],end_XZ[0,2]]),-EE_virtual_angs[2])])
              # 
              fk_targets = np.hstack([rot_XZ_on_Y(np.array([mid_X - base_X, mid_Z - base_Z]),-EE_virtual_angs[2]), rot_XZ_on_Y(np.array([end_X - base_X, end_Z - base_Z]),-EE_virtual_angs[2])])
              print("fk_targets: ", fk_targets)
              theta_guess = np.array([1e-3,1e-3]) # TODO - replace this to get estimate from optimisation solution - need to include in generated sequence.csv. Only on first run, then use next estimate
            #   # Curvature from IK iteration
              theta_extracted, convergence = find_curvature(p_vals, theta_guess, target_evaluators, fk_targets, 0.005)
              
              print("Phi_seq:", Phi_seq)
              # Calculate J^-1
              # J = eval_J_end_wrt_base(np.array([theta_extracted[0], theta_extracted[1], base_XZ[0,0], base_XZ[0,2], angs[2]]), p_vals)

                # def eval_J_end_wrt_base(x, z, phi, p_vals):
              # TODO ask Seb if these input arguments are need or the one in the function that is defined
              fk = eval_fk(np.array([theta_extracted[0], theta_extracted[1], base_X, base_Z, EE_virtual_angs[2]]), p_vals, 1.0, 0.0)
              print("fk: ", fk)
              J = eval_J_end_wrt_base(np.array([theta_extracted[0], theta_extracted[1], base_X, base_Z, EE_virtual_angs[2]]), p_vals)
              # J = eval_J_end_wrt_base(base_X, base_Z, Phi_seq[1], p_vals)

            #   # Calc new manipulator pose (pseudo code)
              # step direction and magnitude proportional to the gain delta
              manipulator_step = J^-1 * (np.array([Goals_X[i], Goals_Z[i], Goals_Alpha[i]]) - np.array([end_XZ[0,0], end_XZ[0,2], alpha_ang])) * delta
              # the using moveit update the endeffector position based the error and gain  
              plan = plan_to_cart(X_current + manipulator_step[0], object_Y_plane, Z_current + manipulator_step[1], pi, 0, Phi_current + manipulator_step[2])
              # execute_plan(plan)
              # Get new image, robot state, user input ...
              ##
            
            #   # Save the last image into a separate directory (eg ./images_feedback...)

            # Save data
            X_EE.append(trans[0])
            Z_EE.append(trans[2])
            Phi_EE.append(angs[2])
            X_base_meas.append(base_XZ[0,0])
            Z_base_meas.append(base_XZ[2,0])
            X_mid_meas.append(mid_XZ[0,0])
            Z_mid_meas.append(mid_XZ[2,0])
            X_end_meas.append(end_XZ[0,0])
            Z_end_meas.append(end_XZ[2,0])
            U_base.append(base_UV[0])
            V_base.append(base_UV[1])
            U_mid.append(mid_UV[0])
            V_mid.append(mid_UV[1])
            U_end.append(end_UV[0])
            V_end.append(end_UV[1])
            U_ang_start.append(base_ang_start_UV[0])
            V_ang_start.append(base_ang_start_UV[1])
            U_ang_end.append(base_ang_end_UV[0])
            V_ang_end.append(base_ang_end_UV[1])
            X_ang_start.append(base_ang_start_XZ[0,0])
            Z_ang_start.append(base_ang_start_XZ[2,0])
            X_ang_end.append(base_ang_end_XZ[0,0])
            Z_ang_end.append(base_ang_end_XZ[2,0])
            Base_angle.append(base_ang)
            
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
