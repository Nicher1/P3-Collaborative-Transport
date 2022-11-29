import URBasic
import numpy as np
import time
from URBasic import kinematic
from URBasic.kinematic import Invkine_manip, Tran_Mat2Pose, Pose2Tran_Mat, Inverse_kin
from ikpy import chain

matStart = np.array([[-0.000000, 0.000000, 1.000000, 663.799988],
             [-1.000000, -0.000000, -0.000000, -76.487640],
             [0.000000, -1.000000, 0.000000, 856.400024],
             [0.000000, 0.000000, 0.000000, 1.000000]])
matEnd = np.array([[-0.000000, 0.000000, 1.000000, 663.800000],
           [-1.000000, -0.000000, -0.000000, -163.900000],
           [0.000000, -1.000000, 0.000000, 856.400000],
           [0.000000, 0.000000, 0.000000, 1.000000]])
jointStart = [8.889723, -88.252324, -91.719105, -0.028571, 81.110277, -0.000000]
jointStart = np.radians(jointStart)
jointEnd = [0.000000, -90.000000, -90.000000, 0.000000, 90.000000, -0.000000]
jointEnd = np.radians(jointEnd)
print(jointEnd)

poseEnd = Tran_Mat2Pose(matEnd)
invKinematics = Inverse_kin(matEnd, jointStart)
print(invKinematics)
