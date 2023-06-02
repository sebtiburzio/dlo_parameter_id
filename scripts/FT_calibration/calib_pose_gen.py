f = open("generated_poses.txt", 'w')

pose_id = 0

# # Full R^3 Euler XYZ
# Rangs = [0.0, 0.7854, 1.571, 2.356, 3.142, 3.927, 4.712, 5.498]
# RPangs = [0.7854, 2.356, 3.927, 5.498]
# PRangs = [-0.7854, 0.7854]
# Pangs = [0.0, 0.7854, 1.571, 2.356, 3.142, 3.927, 4.712, 5.498]
# Yangs = [0.0, 1.571, 3.142, 4.712]
# # Around X axis
# for R in Rangs:
#     for Y in Yangs:
#         f.write("pose" + str(pose_id) + ": [0.3, 0.0, 0.55, " + str(R) + ", " + str(0.0) + ", " + str(Y) + "]\n")
#         pose_id += 1
# # Around Y axis
# for P in Pangs:
#     for Y in Yangs:
#         f.write("pose" + str(pose_id) + ": [0.3, 0.0, 0.55, " + str(R) + ", " + str(P) + ", " + str(0.0) + "]\n")
#         pose_id += 1
# # Around Z axis
# for Y in Yangs:
#     f.write("pose" + str(pose_id) + ": [0.3, 0.0, 0.55, " + str(1.571) + ", " + str(0.7854) + ", " + str(0.0) + "]\n")
#     pose_id += 1
# for Y in Yangs:
#     f.write("pose" + str(pose_id) + ": [0.3, 0.0, 0.55, " + str(1.571) + ", " + str(-0.7854) + ", " + str(0.0) + "]\n")
#     pose_id += 1
# # Combined X & Y axes
# for R in RPangs:
#     for P in PRangs:
#         for Y in Yangs:
#             f.write("pose" + str(pose_id) + ": [0.3, 0.0, 0.55, " + str(R) + ", " + str(P) + ", " + str(Y) + "]\n")
#             pose_id += 1

# Euler YXZ, bottom cone of R^3 sphere & 4 around final Z
Yangs = [2.356, 3.142, 3.927]
Xangs = [-0.7854, 0.0, 0.7854]
Zangs = [0.0, 1.571, 3.142, 4.712]
for Y in Yangs:
    for X in Xangs:
        for Z in Zangs:
            f.write("pose" + str(pose_id) + ": [0.3, 0.0, 0.55, " + str(Y) + ", " + str(X) + ", " + str(Z) + "]\n")
            pose_id += 1

f.close()