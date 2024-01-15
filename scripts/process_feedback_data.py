import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 


from generated_functions.floating.floating_base_functions import eval_fk, eval_J_end_wrt_base
from generated_functions.fixed.fixed_base_functions import eval_midpt, eval_endpt, eval_J_midpt, eval_J_endpt


def rotate(x, theta):
    rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], 
                          [np.sin(theta), np.cos(theta)]])
    out = rotMatrix @ x
    return out

def rotate_around_point_lowperf(point, radians, origin=(0, 0)):
    """Rotate a point around a given point.
    
    I call this the "low performance" version since it's recalculating
    the same values more than once [cos(radians), sin(radians), x-ox, y-oy).
    It's more readable than the next function, though.
    """
    x, y = point
    ox, oy = origin

    # qx = ox + math.cos(radians) * (x - ox) + math.sin(radians) * (y - oy)
    # qy = oy + -math.sin(radians) * (x - ox) + math.cos(radians) * (y - oy)


    qx = ox + np.cos(radians) * (x - ox) + np.sin(radians) * (y - oy)
    qy = oy + -np.sin(radians) * (x - ox) + np.cos(radians) * (y - oy)

    return qx, qy


data = pd.read_csv('./data/12-07/test/sequence_results.csv')
targets = pd.read_csv('./data/12-07/test/sequence.csv', header=None)
# preprocess tip and end effector error
idx = range(len(data['ts']))
print("idx: ", idx)
arrow_lengths_X = data['X_end_meas'][:-1] - data['X_end_meas'][2:]
arrow_lengths_Z = data['Z_end_meas'][:-1] - data['Z_end_meas'][2:]
print("data['X_end_meas'][:-1]: ", data['X_end_meas'][:-1])
print("data['X_end_meas'][1:]: ", data['X_end_meas'][2:])

print("arrow lengths X: ", arrow_lengths_X)
print("arrow lengths Z: ", arrow_lengths_Z)

# plot the time series of the error to the goal
fig, ax = plt.subplots()


ax.scatter(data['X_end_meas'], data['Z_end_meas'], label='Measured object end point')
# add the numbers to the data points 
[plt.text(data['X_end_meas'][i], data['Z_end_meas'][i], str(idx[i]+1)) for i in idx]
print(type(data['X_end_meas']))
print("length[1:]: ", len(data['X_end_meas'][1:]))
print("length[:-1]: ", len(data['X_end_meas'][:-1]))

ax.set_xlabel("x (m)")
ax.set_ylabel("z (m)")
ax.set_aspect('equal', adjustable='box')

for i in range(1, len(idx)-1):
    # ax.arrow(data['X_end_meas'][i], data['Z_end_meas'][i], data['X_end_meas'][i] - data['X_end_meas'][i+1],data['Z_end_meas'][i] - data['Z_end_meas'][i+1])
    ax.arrow(data['X_end_meas'][i], data['Z_end_meas'][i], arrow_lengths_X[i], arrow_lengths_Z[i], length_includes_head=True)

# Add the goal
# goal_X = targets[]

print("targets", targets)
Endpt_X = targets.iloc[0, 8]
Endpt_Z = targets.iloc[0, 9]
# Endpt_Alpha = targets.iloc[:, 10]

Goals_X = targets.iloc[0,5]
print("Goals_X", Goals_X)
Goals_Z = targets.iloc[0,6]
print("Goals_Z", Goals_Z)
Goals_Alpha = targets.iloc[0, 7]

# ax.scatter(Endpt_X, Endpt_Z, label='End point target', color='red')
ax.scatter(Goals_X, Goals_Z, label='End point target', color='red')
plt.text(Goals_X, Goals_Z, "Target end point")


# Add angle lines to the image
# tip_angle_pts = [rotate(np.array([data['X_end_meas'][i]+0.001, data['Z_end_meas'][i]]), data['Base_angle'][i]+np.pi/4) for i in idx]
# tip_angle_pts = [rotate(np.array([data['X_end_meas'][i]+0.001, data['Z_end_meas'][i]]), data['Base_angle'][i]) for i in idx]
tip_angle_pts = [rotate_around_point_lowperf(np.array([data['X_end_meas'][i]+0.005, data['Z_end_meas'][i]]), data['Base_angle'][i] -np.pi/2, origin=np.array([data['X_end_meas'][i], data['Z_end_meas'][i]]) ) for i in idx]



tip_angle_pts_X = [tip_angle_pts[i][0] for i in idx]
tip_angle_pts_Z = [tip_angle_pts[i][1] for i in idx]
# print("tip_angle_pts: ", tip_angle_pts)

# print("tip_angle_pts X:", tip_angle_pts_X)
# [ax.plot(data['X_end_meas'][i], data['Z_end_meas'][i], tip_angle_pts[i][0], tip_angle_pts[i][1], color="black") for i in idx]
# [ax.plot(data['X_end_meas'][i], data['Z_end_meas'][i], tip_angle_pts_X[i], tip_angle_pts_Z[i], color="black") for i in idx]

[ax.plot(np.array([data['X_end_meas'][i], tip_angle_pts_X[i]]), np.array([data['Z_end_meas'][i], tip_angle_pts_Z[i]]), color="black") for i in idx]

# [ax.plot(np.array([data['X_end_meas'][i], data['Z_end_meas'][i]]), np.array([tip_angle_pts_X[i], tip_angle_pts_Z[i]]), color="black") for i in idx]

# ax.plot(data['X_end_meas'], data['Z_end_meas'], tip_angle_pts_X, tip_angle_pts_Z, color="black")

# Plot the goal target angle
# Goal_angle_pts = rotate(np.array([Goals_X 0.005, Goals_Z]), targets[0, ]
# ax.plot(Goals_X, Goals_Z, Goal_angle_pts[0], Goal_angle_pts[1])

plt.show()

error = [np.linalg.norm(np.array([data['X_end_meas'][i], data['Z_end_meas'][i]]) - np.array([Goals_X,  Goals_Z])) for i in idx]
error_X = [np.array([Goals_X] - data['X_end_meas'][i]) for i in idx]
error_Z = [np.array([Goals_Z] - data['Z_end_meas'][i]) for i in idx]
print("error: ", error)
plt.plot(error_X, label = 'x error')
plt.plot(error_Z, label = 'z error')
plt.show()

plt.plot(data['X_EE'])
plt.plot(data['Z_EE'])
plt.show()


# Jabobian_output = [eval_J_end_wrt_base(np.array([theta_extracted[0], theta_extracted[1], base_X, base_Z, EE_virtual_angs[2]]), p_vals)]

Jabobian_output = [eval_J_end_wrt_base(np.array(['Theta0'][i][0]), np.array(['Theta0'][i][1]), data['base_X'][i], data['base_Z'][i], data['base_ang'][i], p_vals)]


# plt.plot(error_X)
# plt.show()
