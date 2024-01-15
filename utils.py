import numpy as np
import matplotlib.pyplot as plt

import pandas as pd 

def rot_XZ_on_Y(XZs,angles):
    # HACK - the angles are -ve because R_angles needs to be transposed for einsum to work
    # I don't know how to get einsum to work otherwise
    R_angles = np.array([[np.cos(-angles), np.sin(-angles)], 
                        [-np.sin(-angles), np.cos(-angles)]]).T
    if len(XZs.shape) == 1:
        return R_angles@XZs
    else:
        return np.einsum('ijk,ik->ij', R_angles, XZs)

def get_FK(p_vals,q_repl,f_fk,num_pts=21):
    s_evals = np.linspace(0,1,num_pts)
    FK_evals = np.zeros((s_evals.size,2,1))
    for i_s in range(s_evals.size):
       FK_evals[i_s] = np.array(f_fk(q_repl,p_vals,s_evals[i_s],0.0))
    return FK_evals.squeeze()

# Plot FK based on theta config and optionally an fk target for comparison
def plot_FK(p_vals,q_repl,f_fk,fk_targets=None):
    FK_evals = get_FK(p_vals,q_repl,f_fk)
    fig, ax = plt.subplots()
    ax.plot(FK_evals[:,0],FK_evals[:,1],'tab:orange')
    ax.scatter(FK_evals[10,0],FK_evals[10,1],s=2,c='m',zorder=2.5)
    ax.scatter(FK_evals[-1,0],FK_evals[-1,1],s=2,c='m',zorder=2.5)
    plt.xlim(FK_evals[0,0]-1.1*p_vals[2],FK_evals[0,0]+1.1*p_vals[2])
    plt.ylim(FK_evals[0,1]-1.1*p_vals[2],FK_evals[0,1]+1.1*p_vals[2])

    if fk_targets is not None:
        plt.scatter(0,0,c='tab:red',marker='+')
        plt.scatter(fk_targets[0],fk_targets[1],c='tab:green',marker='+')
        plt.scatter(fk_targets[2],fk_targets[3],c='tab:blue',marker='+')

    fig.set_figwidth(8)
    ax.set_aspect('equal','box')
    ax.grid(True)
    plt.show()

def plot_fk_targets(fk_targets,i):
    plt.scatter(0,0,c='tab:red',marker='+')
    plt.scatter(fk_targets[i,0],fk_targets[i,1],c='tab:green',marker='+')
    plt.scatter(fk_targets[i,2],fk_targets[i,3],c='tab:blue',marker='+')
    plt.axis('equal')
    plt.grid(True)

# target_evaluators = [eval_midpt, eval_endpt, eval_J_midpt, eval_J_endpt]
def find_curvature(p_vals,theta_guess,target_evaluators,fk_target,epsilon=0.01,max_iterations=10):  
    error_2norm_last = np.inf
    for i in range(max_iterations):
        error = (np.vstack([target_evaluators[0](theta_guess,p_vals), target_evaluators[1](theta_guess,p_vals)]) - fk_target.reshape(4,1))
        error_2norm = np.linalg.norm(error)
        if error_2norm < epsilon:
            print("Converged after " + str(i) + " iterations")
            return theta_guess, True
        else:
            if np.isclose(error_2norm, error_2norm_last):
                print("Error stable after iteration " + str(i))
                return theta_guess, False
            elif error_2norm > error_2norm_last:
                print("Error increasing after iteration " + str(i))
                return theta_guess_last, False
            else:
                theta_guess_last = theta_guess
                error_2norm_last = error_2norm
                J = np.vstack([target_evaluators[2](theta_guess, p_vals), target_evaluators[3](theta_guess, p_vals)])
                theta_guess = theta_guess - (np.linalg.pinv(J)@error).squeeze()
    print("Max iterations reached (check why)")
    return theta_guess, False



def rotate_around_point(point, radians, origin=(0, 0)):
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


# def plot_feedback_process(data_path, targets_path):
def plot_feedback_process(data_path, targets_path):
    
    # data = pd.read_csv('./sequence_results.csv')
    # targets = pd.read_csv('./sequence.csv', header=None)
    data = pd.read_csv(data_path)
    targets = pd.read_csv(targets_path, header=None)
    # preprocess tip and end effector error

    idx = range(len(data['ts']))
    print("idx: ", idx)
    # arrow_lengths_X = data['X_end_meas'][:-1] - data['X_end_meas'][2:]
    # arrow_lengths_Z = data['Z_end_meas'][:-1] - data['Z_end_meas'][2:]
    # print("data['X_end_meas'][:-1]: ", data['X_end_meas'][:-1])
    # print("data['X_end_meas'][1:]: ", data['X_end_meas'][2:])
    #
    # print("arrow lengths X: ", arrow_lengths_X)
    # print("arrow lengths Z: ", arrow_lengths_Z)

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
    tip_angle_pts = [rotate_around_point(np.array([data['X_end_meas'][i]+0.005, data['Z_end_meas'][i]]), data['Base_angle'][i] -np.pi/2, origin=np.array([data['X_end_meas'][i], data['Z_end_meas'][i]]) ) for i in idx]



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

def plot_error_progress(data):
    print("error plot")


def plot_delta_x_progress(data):

    print("delat_x")


