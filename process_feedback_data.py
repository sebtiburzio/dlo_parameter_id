import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 



# from generated_functions.floating.floating_base_functions import eval_fk, eval_J_end_wrt_base
# from generated_functions.fixed.fixed_base_functions import eval_midpt, eval_endpt, eval_J_midpt, eval_J_endpt

from acdlo import floating_base
from acdlo import static_base


target_evaluators = [static_base.eval_midpt, static_base.eval_endpt, static_base.eval_J_midpt, static_base.eval_J_endpt]
from utils import rot_XZ_on_Y, get_FK, find_curvature, plot_feedback_process


# from generated_functions.floating.floating_base_functions import eval_fk, eval_J_end_wrt_base
# from generated_functions.fixed.fixed_base_functions import eval_midpt, eval_endpt, eval_J_midpt, eval_J_endpt


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


data = pd.read_csv('data/12-19/test/sequence_results_1.csv')
# data = pd.read_csv('data/12-18/test/sequence_results.csv')
targets = pd.read_csv('data/12-19/test/sequence.csv', header=None)
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
plt.title("Feedback results")


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

ax.set_xlabel("x (m)")
ax.set_ylabel("z (m)")
plt.legend()
plt.show()

error = [np.linalg.norm(np.array([data['X_end_meas'][i], data['Z_end_meas'][i]]) - np.array([Goals_X,  Goals_Z])) for i in idx]
error_X = [np.array([Goals_X] - data['X_end_meas'][i]) for i in idx]
error_Z = [np.array([Goals_Z] - data['Z_end_meas'][i]) for i in idx]
print("error: ", error)
plt.plot(error_X, label = 'x error')
plt.plot(error_Z, label = 'z error')
# ax.set_title(label="measurement error at tip")
plt.title(label="measurement error at tip")
plt.legend()

# ax.set_xlabel("x (m)")
# ax.set_ylabel("z (m)")

plt.xlabel("x (m)")
plt.ylabel("z (m)")
plt.show()

plt.plot(data['X_EE'])
plt.plot(data['Z_EE'])
plt.title("End Effector position over time")
# ax.set_xlabel("x (m)")
# ax.set_ylabel("z (m)")
plt.xlabel("x (m)")
plt.ylabel("z (m)")
plt.legend()
plt.show()


# Jabobian_output = [eval_J_end_wrt_base(np.array([theta_extracted[0], theta_extracted[1], base_X, base_Z, EE_virtual_angs[2]]), p_vals)]



with np.load('/home/mossy/Documents/Delft/MSc_students/Seb/dlo_parameter_id/object_parameters/black_weighted.npz') as obj_params:
  p_vals = list(obj_params['p_vals']) # cable properties: mass (length), mass (end), length, diameter

# print("np.array(data['Theta0'][i][0]): ", np.array(data['Theta0'][1][0]))
# print("np.array(data['Theta0'][i][0]): ", np.array(data['Theta0']).shape)
# print("np.array(data['Theta0'][i][0]): ", np.array(data['Theta0'][1]))
#
# J = eval_J_end_wrt_base(np.array([theta_extracted[0], theta_extracted[1], base_X, base_Z, EE_virtual_angs[2]]), p_vals)
Jacobian_output = [floating_base.eval_J_end_wrt_base(np.array([data['Theta0'][i], data['Theta1'][i], data['X_base_meas'][i], data['Z_base_meas'][i], data['Base_angle'][i]]), p_vals) for i in idx]

Jacobian_rank = [np.linalg.matrix_rank(Jacobian_output[i], tol=0.001) for i in idx] 
print("Jacobian_rank: ", Jacobian_rank)

j_pinv = np.linalg.inv(Jacobian_output)
# Jacobian_pinv_rank = [np.linalg.matrix_rank(Jacobian_output[i], tol=0.001) for i in idx] 
print("j_pinv: ", j_pinv)
print("np.array([data['X_end_meas'][:, i]): ", np.array([data['X_end_meas'][0]]))
print("np.array([data['X_end_meas']): ", np.array([data['X_end_meas']]))
input_error = np.array([np.array([Goals_X, Goals_Z, Goals_Alpha]) - np.array([data['X_end_meas'][i], data['Z_end_meas'][i], data['Base_angle'][i]]) for i in idx])
print("input_error: ", input_error)
# print("input_error[0]: ", input_error[:, 1])
# print("input_error[0]: ", input_error.shape)
plt.plot(input_error[:, 0], label='x error')
plt.plot(input_error[:, 1], label='z error')
plt.plot(input_error[:, 2], label='alpha error')
plt.legend()
ax.set_aspect('auto', adjustable='box')
plt.title("Measurement error")
# ax.set_xlabel("x (m)")
# ax.set_ylabel("z (m)")

plt.xlabel("x (m)")
plt.ylabel("z (m)")

plt.show()



# manipulator_step = j_pinv @ (np.array([Goals_X[i], Goals_Z[i], Goals_Alpha[i]]) - np.array([end_X, end_Z, base_ang])) * k_gain
k_gain = 1.0
manipulator_step = np.array([j_pinv[i] @ input_error[i, :] * k_gain for i in idx])
print("manipulator_step: ", manipulator_step)
# print("j_pinv[0]: ", j_pinv[0])
plt.plot(manipulator_step[:, 0], label='x error', color='red')
plt.plot(manipulator_step[:, 1], label='z error', color='blue')
plt.plot(manipulator_step[:, 2], label='alpha error', color='green')
plt.legend()
ax.set_aspect('auto', adjustable='box')
# ax.set_xlabel("x (m)")
ax.set_ylabel("(m)")
plt.title("delta x: feedback input")
plt.legend()
# ax.set_xlabel("x (m)")
# ax.set_ylabel("z (m)")
plt.xlabel("x (m)")
plt.ylabel("z (m)")
plt.show()

# manipulator_step = j_pinv @ (np.array([Goals_X[i], Goals_Z[i], Goals_Alpha[i]]) - np.array([end_X, end_Z, base_ang])) * k_gain

# print("Jacobian output: ", Jacobian_output)

# plt.plot(error_X)
# plt.show()
