
#%%

from copy import copy
from pathlib import Path
from sys import path
import csv
import numpy as np
from matplotlib import pyplot as plt

#%%
# Path to the build directory including a file similar to 'ruckig.cpython-37m-x86_64-linux-gnu'.
build_path = Path(__file__).parent.absolute().parent / 'build'
path.insert(0, str(build_path))

from ruckig import InputParameter, OutputParameter, Result, Ruckig


# Create instances: the Ruckig OTG as well as input and output parameters
otg = Ruckig(3, 0.001)  # DoFs, control cycle rate, maximum number of intermediate waypoints for memory allocation
inp = InputParameter(3)  # DoFs
out = OutputParameter(3, 10)  # DoFs, maximum number of intermediate waypoints for memory allocation

inp.current_position = [0.0, 0.0, 0.0]
inp.current_velocity = [0, 0.0, 0]
inp.current_acceleration = [0, 0.0, 0]

inp.target_position = [0.0, 0.1, 0.0]
inp.target_velocity = [0.0, 0, 0.0]
inp.target_acceleration = [0, 0.0, 0.0]

inp.max_velocity = [0.1, 0.1, 0.5]
inp.max_acceleration = [4.5/2, 4.5/2, 8.5/2]
inp.max_jerk = [4500.0, 4500.0, 8500.0]


print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))

# Generate the trajectory within the control loop
first_output, out_list = None, []
res = Result.Working
while res == Result.Working:
    res = otg.update(inp, out)

    #print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
    out_list.append(copy(out))

    out.pass_to_input(inp)

    if not first_output:
        first_output = copy(out)

print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')

t = [o.time for o in out_list]
X = [o.new_position[0] for o in out_list]
Z =[o.new_position[1] for o in out_list]
Phi = [o.new_position[2] for o in out_list]
dX = [o.new_velocity[0] for o in out_list]
dZ = [o.new_velocity[1] for o in out_list]
dPhi = [o.new_velocity[2] for o in out_list]
ddX = [o.new_acceleration[0] for o in out_list]
ddZ = [o.new_acceleration[1] for o in out_list]
ddPhi = [o.new_acceleration[2] for o in out_list]

#%%
plt.figure(figsize=(10, 15))
plt.subplot(3, 1, 1)
plt.plot(t, X, label='position')
plt.plot(t, dX, label='velocity')
plt.plot(t, ddX, label='acceleration')
plt.legend()
plt.subplot(3, 1, 2)
plt.plot(t, Z, label='position')
plt.plot(t, dZ, label='velocity')
plt.plot(t, ddZ, label='acceleration')
plt.legend()
plt.subplot(3, 1, 3)
plt.plot(t, Phi, label='position')
plt.plot(t, dPhi, label='velocity')
plt.plot(t, ddPhi, label='acceleration')
plt.legend()
plt.figure()
plt.plot(X,Z)
for i in np.linspace(0,len(Phi)-1,10).astype(int):
    plt.plot([X[i],X[i]+0.01*np.sin(Phi[i])],[Z[i],Z[i]-0.01*np.cos(Phi[i])], c='r')
plt.axis('equal')

# %%
with open('./traj.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter='\t')
    #writer.writerow(['ts', 'base_pos_x', 'base_pos_z', 'mid_pos_x', 'mid_pos_z', 'end_pos_x', 'end_pos_z'])
    for n in range(len(t)):
        writer.writerow([X[n], Z[n], Phi[n]])
# %%
