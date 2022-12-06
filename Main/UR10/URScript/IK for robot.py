from spatialmath import SE3

Tep = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
print(Tep)
sol = robot.ik_lm_chan(Tep)         # solve IK
print(sol)