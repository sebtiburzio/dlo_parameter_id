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
def plot_vis():
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(autoscale_on=False)
    ax.set_aspect('equal')
    ax.set_xlim(0,img.shape[1])
    ax.set_ylim(0,img.shape[0])

    ax.imshow(img)

    EE_UV = P@EE_XYZ
    EE_UV = EE_UV/EE_UV[2]
    marked_UV = P@marked_XYZ
    marked_UV = marked_UV/marked_UV[2]

    ax.scatter(EE_UV[0],EE_UV[1],s=5,c='red',zorder=2.5)
    ax.scatter(marked_UV[:,0],marked_UV[:,1],s=50,c='red',zorder=2.5)

    # Z-plane grid
    grid_btm = P@np.array([np.linspace(-0.2,0.2,5).T,-0.2*np.ones(5),np.zeros(5),np.ones(5)])
    grid_btm = grid_btm/grid_btm[2,:].reshape(1,-1)
    grid_top = P@np.array([np.linspace(-0.2,0.2,5).T,0.2*np.ones(5),np.zeros(5),np.ones(5)])
    grid_top = grid_top/grid_top[2,:].reshape(1,-1)
    grid_left = P@np.array([-0.2*np.ones(5),np.linspace(-0.2,0.2,5).T,np.zeros(5),np.ones(5)])
    grid_left = grid_left/grid_left[2,:].reshape(1,-1)
    grid_right = P@np.array([0.2*np.ones(5),np.linspace(-0.2,0.2,5).T,np.zeros(5),np.ones(5)])
    grid_right = grid_right/grid_right[2,:].reshape(1,-1)
    for i in range(5):
        ax.plot([grid_btm[0,i],grid_top[0,i]],[grid_btm[1,i],grid_top[1,i]],lw=1,c='aqua')
        ax.plot([grid_left[0,i],grid_right[0,i]],[grid_left[1,i],grid_right[1,i]],lw=1,c='aqua')
    # Y-plane grid
    grid_btm = P@np.array([np.linspace(0,1.0,11).T,np.zeros(11),np.zeros(11),np.ones(11)])
    grid_btm = grid_btm/grid_btm[2,:].reshape(1,-1)
    grid_top = P@np.array([np.linspace(0,1.0,11),np.zeros(11),np.ones(11),np.ones(11)])
    grid_top = grid_top/grid_top[2,:].reshape(1,-1)
    grid_left = P@np.array([np.zeros(11),np.zeros(11),np.linspace(0,1.0,11),np.ones(11)])
    grid_left = grid_left/grid_left[2,:].reshape(1,-1)
    grid_right = P@np.array([np.ones(11),np.zeros(11),np.linspace(0,1.0,11),np.ones(11)])
    grid_right = grid_right/grid_right[2,:].reshape(1,-1)
    for i in range(11):
        ax.plot([grid_btm[0,i],grid_top[0,i]],[grid_btm[1,i],grid_top[1,i]],lw=1,c='lime')
        ax.plot([grid_left[0,i],grid_right[0,i]],[grid_left[1,i],grid_right[1,i]],lw=1,c='lime')
    # Robot base links
    base_links = P@np.array([[0,0,0,1],[0,0,0.333,1],[0,-0.15,0.333,1],[0,0.15,0.333,1]]).T
    base_links = base_links/base_links[2,:].reshape(1,-1)
    ax.plot(base_links[0,:],base_links[1,:],lw=3,c='slategrey')


#%%

bridge = CvBridge()

bag = rosbag.Bag('./calib_test.bag', "r")

save_dir = './'
print(save_dir)

#%%

# Get the first image
for topic, msg, timestamp in bag.read_messages(topics=['camera/color/image_raw/compressed','/camera/color/image_raw/compressed']):
    img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
    break

# Get the first EE position
for topic, msg, timestamp in bag.read_messages(topics='/franka_state_controller/franka_states'):
    EE_XYZ = np.array([msg.O_T_EE[12], msg.O_T_EE[13], msg.O_T_EE[14], msg.O_T_EE[15]])
    break

# Get the intrinsic camera matrix
for topic, msg, timestamp in bag.read_messages(topics='/camera/color/camera_info'):
    K_cam = np.array(msg.K).reshape(3,3)
    break

# Get the extrinsic and projection matrices
for file in os.listdir():
    if file.startswith('calib') and file.endswith('.launch'):
        R_line = np.loadtxt(file, dtype='str', delimiter=' ', skiprows=5, max_rows=1)
        R_cam = R.from_quat([float(R_line[11]), float(R_line[12]), float(R_line[13]), float(R_line[14])]).as_matrix() # cam to base frame
        T_cam = np.array([[float(R_line[6][6:])],[float(R_line[7])],[float(R_line[8])]])
        E_base = np.hstack([R_cam, T_cam]) # cam to base frame
        E_cam = np.hstack([R_cam.T, -R_cam.T@T_cam]) # base to cam frame
        P = K_cam@E_cam

# Set up any markerd points to compare
marked_XYZ = np.array([[0.1,0.0,0.0,1.0],
                       [0.2,0.0,0.0,1.0],
                       [0.3,0.0,0.0,1.0],
                    ]).T 

# Plot the visualistion of known geometry on the current calibration
plot_vis()

T_adj = T_cam
R_adj = R_cam

#%%
# Adjust the extrinsic calibration
adj_X = 0.0
adj_Y = 0.0
adj_Z = 0.0
adj_R = 0.0
adj_P = 0.0
adj_Y = 0.0
T_adj = T_adj + np.array([[adj_X],[adj_Y],[adj_Z]])
R_adj = R.from_euler('xyz', [adj_R, adj_P, adj_Y]).as_matrix()@R_adj
E_base_adj = np.hstack([R_adj, T_adj]) 
E_cam_adj = np.hstack([R_adj.T, -R_adj.T@T_adj])
P_adj = K_cam@E_cam_adj
plot_vis()

#%%
# Save the original and adjusted calibrations
np.savez(save_dir + '/TFs.npz', P=P, E_base=E_base, E_cam=E_cam, K_cam=K_cam)
np.savez(save_dir + '/TFs_adj.npz', P=P_adj, E_base=E_base_adj, E_cam=E_cam_adj, K_cam=K_cam)
