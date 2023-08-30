#%%

import os
import cv2
import matplotlib.pyplot as plt
import rosbag
from cv_bridge import CvBridge
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

    EE_UV = P_adj@EE_XYZ
    EE_UV = EE_UV/EE_UV[2]
    marked_UV = P_adj@marked_XYZ
    marked_UV = marked_UV/marked_UV[2]

    ax.scatter(EE_UV[0],EE_UV[1],s=15,c='red',zorder=2.5)
    ax.scatter(marked_UV[0,:],marked_UV[1,:],s=5,c='yellow',zorder=2.5)

    # # Z-plane grid
    # grid_btm = P_adj@np.array([np.linspace(-0.2,0.2,5).T,-0.2*np.ones(5),np.zeros(5),np.ones(5)])
    # grid_btm = grid_btm/grid_btm[2,:].reshape(1,-1)
    # grid_top = P_adj@np.array([np.linspace(-0.2,0.2,5).T,0.2*np.ones(5),np.zeros(5),np.ones(5)])
    # grid_top = grid_top/grid_top[2,:].reshape(1,-1)
    # grid_left = P_adj@np.array([-0.2*np.ones(5),np.linspace(-0.2,0.2,5).T,np.zeros(5),np.ones(5)])
    # grid_left = grid_left/grid_left[2,:].reshape(1,-1)
    # grid_right = P_adj@np.array([0.2*np.ones(5),np.linspace(-0.2,0.2,5).T,np.zeros(5),np.ones(5)])
    # grid_right = grid_right/grid_right[2,:].reshape(1,-1)
    # for i in range(5):
    #     ax.plot([grid_btm[0,i],grid_top[0,i]],[grid_btm[1,i],grid_top[1,i]],lw=1,c='aqua')
    #     ax.plot([grid_left[0,i],grid_right[0,i]],[grid_left[1,i],grid_right[1,i]],lw=1,c='aqua')
    # Y-plane grid
    grid_btm = P_adj@np.array([np.linspace(0,1.0,11).T,np.zeros(11),np.zeros(11),np.ones(11)])
    grid_btm = grid_btm/grid_btm[2,:].reshape(1,-1)
    grid_top = P_adj@np.array([np.linspace(0,1.0,11),np.zeros(11),np.ones(11),np.ones(11)])
    grid_top = grid_top/grid_top[2,:].reshape(1,-1)
    grid_left = P_adj@np.array([np.zeros(11),np.zeros(11),np.linspace(0,1.0,11),np.ones(11)])
    grid_left = grid_left/grid_left[2,:].reshape(1,-1)
    grid_right = P_adj@np.array([np.ones(11),np.zeros(11),np.linspace(0,1.0,11),np.ones(11)])
    grid_right = grid_right/grid_right[2,:].reshape(1,-1)
    for i in range(11):
        ax.plot([grid_btm[0,i],grid_top[0,i]],[grid_btm[1,i],grid_top[1,i]],lw=1,c='lime')
        ax.plot([grid_left[0,i],grid_right[0,i]],[grid_left[1,i],grid_right[1,i]],lw=1,c='lime')
    # Robot base links
    base_links = P_adj@np.array([[0,0,0,1],[0,0,0.333,1],[0,-0.15,0.333,1],[0,0.15,0.333,1]]).T
    base_links = base_links/base_links[2,:].reshape(1,-1)
    ax.plot(base_links[0,:],base_links[1,:],lw=3,c='slategrey')
    # Other robot links when in tilted EE home position (joints=[0.0, -0.1, 0.0, -1.0, 0.0, 1.65, 0])
    home_links = P_adj@np.array([[0,0,0.333,1],[0.052,0,0.655,1],[0.052,-0.1,0.655,1],[0.052,0.1,0.655,1],
                                 [0.052,0,0.655,1],[0.305,0,0.956,1],[0.305,0.1,0.956,1],
                                 [0.305,0.0,0.956,1],[0.442,0.0,0.936,1]]).T
    home_links = home_links/home_links[2,:].reshape(1,-1)
    ax.plot(home_links[0,:],home_links[1,:],lw=3,c='slategrey')

#%%

calib_name = 'calib_3008' # The calibration data format is backwards compared to the data collection data format :[
save_dir = './' + calib_name + '/'
bag = rosbag.Bag(save_dir + calib_name + '.bag', "r")
print(save_dir)

#%%

bridge = CvBridge()

# Get the first image
for topic, msg, timestamp in bag.read_messages(topics=['camera/color/image_raw/compressed','/camera/color/image_raw/compressed']):
    img = cv2.cvtColor(bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)
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
for file in os.listdir(save_dir):
    if file == (calib_name + '.launch'):
        R_line = np.loadtxt(save_dir + file, dtype='str', delimiter=' ', skiprows=5, max_rows=1)
        R_cam = R.from_quat([float(R_line[11]), float(R_line[12]), float(R_line[13]), float(R_line[14])]).as_matrix() # cam to base frame
        T_cam = np.array([[float(R_line[6][6:])],[float(R_line[7])],[float(R_line[8])]])
        E_base = np.hstack([R_cam, T_cam]) # cam to base frame
        E_cam = np.hstack([R_cam.T, -R_cam.T@T_cam]) # base to cam frame
        P = K_cam@E_cam

# Init adjusted calib matrices
T_adj = T_cam
R_adj = R_cam
P_adj = P

#%%
# Set up any marked points to compare
marked_XYZ = np.array([
                       [0.154,0.149,0.0,1.0], # FR3 TERI base
                       [0.154,-0.150,0.0,1.0], # FR3 TERI base
                       [-0.238,0.149,0.0,1.0], # FR3 TERI base
                       [-0.238,-0.149,0.0,1.0], # FR3 TERI base
                       [0.154,0.149,-0.03,1.0], # FR3 TERI base
                       [0.154,-0.150,-0.03,1.0], # FR3 TERI base
                       [-0.238,0.149,-0.03,1.0], # FR3 TERI base
                       [-0.238,-0.149,-0.03,1.0], # FR3 TERI base
                       [0.055,0.0,0.14,1.0], # FR3 link0 arrow
                       [0.0715,0.0,0.00135,1.0], # FR3 link0 front edge
                    ]).T

# Plot the visualistion of known geometry on the current calibration
plot_vis()

#%%
# Adjust the extrinsic calibration
adj_X = 0.0
adj_Y = 0.0
adj_Z = 0.0
adj_Roll = 0.0
adj_Pitch = 0.0
adj_Yaw = 0.0
T_adj = T_adj + np.array([[adj_X],[adj_Y],[adj_Z]])
R_adj = R.from_euler('xyz', [adj_Roll, adj_Pitch, adj_Yaw]).as_matrix()@R_adj
E_base_adj = np.hstack([R_adj, T_adj]) 
E_cam_adj = np.hstack([R_adj.T, -R_adj.T@T_adj])
P_adj = K_cam@E_cam_adj
print('X: ' + str(adj_X) + ' Y: ' + str(adj_Y) + ' Z: ' + str(adj_Z) + ' R: ' + str(adj_Roll) + ' P: ' + str(adj_Pitch) + ' Yaw: ' + str(adj_Yaw))
plot_vis()

#%%
# Save the original and adjusted calibrations
np.savez(save_dir + 'TFs.npz', P=P, E_base=E_base, E_cam=E_cam, K_cam=K_cam, T_cam=T_cam, R_cam=R_cam)
np.savez(save_dir + './TFs_adj.npz', P=P_adj, E_base=E_base_adj, E_cam=E_cam_adj, K_cam=K_cam, T_cam=T_adj, R_cam=R_adj)

# %%
TFs_adj = np.load(save_dir + 'TFs_adj.npz')
P_adj = TFs_adj['P']
T_adj = TFs_adj['T_cam']
R_adj = TFs_adj['R_cam']
# %%
