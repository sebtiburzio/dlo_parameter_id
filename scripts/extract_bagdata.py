#!/usr/bin/env python3
#%%

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

#%%
if __name__ == '__main__':
        
    bridge = CvBridge()

    if len(sys.argv) < 2:
        print("Please provide a bag file name wihtout extension as argument!")
        exit()
    bag_name = str(sys.argv[1])
    bag = rosbag.Bag('./../data/' + bag_name + '.bag', "r") # TODO - check if changed path structure works

    save_dir = './../data/' + bag_name
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
    for topic, msg, timestamp in bag.read_messages(topics='/camera/color/image_raw/compressed'):
            if count < 1e9:
                image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv2.imwrite(img_dir + '/' + str(timestamp) + '.jpg', image)
                count += 1
    #%%
    count = 0
    for topic, msg, timestamp in bag.read_messages(topics='/camera/aligned_depth_to_color/image_raw'):
            if count == -1:#1e9:
                depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth_array = np.array(depth, dtype=np.uint16)
                section = 600
                plt.plot(depth_array[section,:])
                plt.show()
                plt.imshow(depth_array, cmap='gray')
                plt.plot([0,1920], [section, section], 'r-')
                #cv2.imwrite(depth_dir + '/' + str(timestamp) + '.png', depth) # TODO - figure out how to save depth image
            count += 1

    #%%
    count = 0
    for topic, msg, timestamp in bag.read_messages(topics='/camera/color/camera_info'):
        if count == 0:
            with open(save_dir + '/color_camera_info.txt', 'w') as f:
                f.write(str(msg))
            count += 1
    #%%
    count = 0
    for topic, msg, timestamp in bag.read_messages(topics='/camera/aligned_depth_to_color/camera_info'):
        if count == 0:
            with open(save_dir + '/depth_camera_info.txt', 'w') as f:
                f.write(str(msg))
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
bag.close()
        