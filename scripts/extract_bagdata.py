#!/usr/bin/env python3
#%%
# Run in the same directory as the bag file

import sys
import os
import csv
import cv2
import matplotlib.pyplot as plt
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

#%%
if __name__ == '__main__':
        
    bridge = CvBridge()

    if len(sys.argv) < 2:
        print("Please provide a bag file name as argument!")
        exit()
    bag_name = str(sys.argv[1])
    if bag_name[-4:] == '.bag':
        bag_name = bag_name[:-4]
    bag = rosbag.Bag('./' + bag_name + '.bag', "r")

    save_dir = './' + bag_name
    print(save_dir)
    img_dir = save_dir + '/images'
    print(img_dir)
    depth_dir = save_dir + '/depth'
    print(depth_dir)
    if os.path.exists(save_dir) == False:
        print('making dirs')
        os.mkdir(save_dir)
        os.mkdir(img_dir)
        os.mkdir(depth_dir)
    #%%
    count = 0
    for topic, msg, timestamp in bag.read_messages(topics=['camera/color/image_raw/compressed','/camera/color/image_raw/compressed']):
            if count < 1e9:
                image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv2.imwrite(img_dir + '/' + str(timestamp) + '.jpg', image)
                count += 1
    #%%
    with open(save_dir + '/EE_pose.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(['ts', 'rxx', 'ryx', 'rzx', 'w1', 'rxy', 'ryy', 'rzy', 'w2', 'rxz', 'ryz', 'rzz', 'w3', 'px', 'py', 'pz', 'w4'])
        for topic, msg, timestamp in bag.read_messages(topics='/franka_state_controller/franka_states'):
            writer.writerow([timestamp, msg.O_T_EE[0], msg.O_T_EE[1], msg.O_T_EE[2], msg.O_T_EE[3], 
                                        msg.O_T_EE[4], msg.O_T_EE[5], msg.O_T_EE[6], msg.O_T_EE[7], 
                                        msg.O_T_EE[8], msg.O_T_EE[9], msg.O_T_EE[10], msg.O_T_EE[11], 
                                        msg.O_T_EE[12], msg.O_T_EE[13], msg.O_T_EE[14], msg.O_T_EE[15]])
    #%%        
    with open(save_dir + '/EE_wrench.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(['ts', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
        for topic, msg, timestamp in bag.read_messages(topics='/ft_compensated'):
            writer.writerow([timestamp, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
    #%%
    for topic, msg, timestamp in bag.read_messages(topics='/camera/color/camera_info'):
        K_cam = np.array(msg.K).reshape(3,3)
        break
    
    for file in os.listdir():
        if file.startswith('calib') and file.endswith('.launch'):
            R_line = np.loadtxt(file, dtype='str', delimiter=' ', skiprows=5, max_rows=1)
            R_cam = R.from_quat([float(R_line[11]), float(R_line[12]), float(R_line[13]), float(R_line[14])]).as_matrix() # cam to base frame
            T_cam = np.array([[float(R_line[6][6:])],[float(R_line[7])],[float(R_line[8])]])
            E_base = np.hstack([R_cam, T_cam]) # cam to base frame
            E_cam = np.hstack([R_cam.T, -R_cam.T@T_cam]) # base to cam frame
            P = K_cam@E_cam
            np.savez(save_dir + '/TFs.npz', P=P, E_base=E_base, E_cam=E_cam, K_cam=K_cam)

#%%
bag.close()
        
#%%
# count = 0
# for topic, msg, timestamp in bag.read_messages(topics='/camera/aligned_depth_to_color/image_raw/compressed'):
#         if count < 1e9:
#             depth = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
#             cv2.imwrite(depth_dir + '/' + str(timestamp) + '.png', depth) # seems to save the data properly if read back in with cv2.imread('', cv2.IMREAD_UNCHANGED)
#             # depth_array = np.array(depth, dtype=np.uint16)
#             # depth_array.astype(np.uint16)
#             # section = 600
#             # print(depth_array)
#             # plt.plot(depth_array[section,:])
#             # plt.show()
#             # plt.imshow(depth_array, cmap='gray')
#             # plt.plot([0,1280], [section, section], 'r-')
#             # plt.show()
#         count += 1
#
# count = 0
# for topic, msg, timestamp in bag.read_messages(topics='/camera/aligned_depth_to_color/camera_info'):
#     if count == 0:
#         with open(save_dir + '/depth_camera_info.txt', 'w') as f:
#             f.write(str(msg))
#         count += 1     