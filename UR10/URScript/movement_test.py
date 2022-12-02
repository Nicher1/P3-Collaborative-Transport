import ikpy as ik
from ikpy.chain import Chain
import numpy as np
import sympy as sp
import math
import scipy
from scipy import linalg
import URBasic
from URBasic.manipulation import *
from URBasic.kinematic import *




def ExampleurScript():
    '''
    This is a small example of how to connect to a Universal Robots robot and use a few simple script commands.
    The scrips available is in general all the scrips from the universal robot script manual,
    and the implementation is intended to follow the Universal Robots manual as much as possible.

    This script can be run connected to a Universal Robot robot (tested at a UR5) or a Universal Robot offline simulator.
    See this example in how to setup an offline simulator:
    https://www.universal-robots.com/download/?option=26266#section16597
    '''

    def moveDirection(startPosition, movementVec,vel,acc):
        '''
        Function for moving the ur10 to a desired position via a 3 dimensional vector.
        The function requires a start position,a vector, and a velocity and acceleration
        '''
        rot = np.array([[0.7071, -0.7071, 0],
                        [0.7071, 0.7071, 0],
                        [0, 0, 1]])
        movementVec = np.matmul(rot, movementVec)
        newPosition = startPosition[:, -1] + np.append(movementVec,0)
        newPosition = np.column_stack((startPosition[:, 0:3], newPosition))
        #robot.movej(pose=kinematic.Tran_Mat2Pose(newPosition), a=acc, v=vel)

    def moveservoj(startPosition,movementVec):
        '''
        Function for moving the ur10 to a desired position via a 3 dimensional vector.
        The function requires a start position,a vector, and a velocity and acceleration
        '''
        rot = np.array([[0.7071, -0.7071, 0],
                        [0.7071, 0.7071, 0],
                        [0, 0, 1]])
        movementVec = np.matmul(rot, movementVec)
        newPosition = startPosition[:, -1] + np.append(movementVec, 0)
        newPosition = np.column_stack((startPosition[:, 0:3], newPosition))
        newPose = Tran_Mat2Pose(newPosition)
        inverseKinematics = Invkine_manip(newPose)

        (inverseKinematics, 0, 0, 0.008, 0.1, 300)
        return newPosition


    ur10Pose = np.array([[0, -0.7071, -0.7071, 0],
                          [0, 0.7071, -0.7071, 0],
                          [1, 0, 0, 0.300],
                          [0, 0, 0, 1]])
    movementVector = np.array([0, 0, 0.4])
    ur10Pose = moveservoj(ur10Pose, movementVector)
    print(ur10Pose)


if __name__ == '__main__':
    ExampleurScript()
