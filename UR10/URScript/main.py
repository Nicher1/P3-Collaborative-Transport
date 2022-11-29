'''
Python 3.x library to control an UR robot through its TCP/IP interfaces
Copyright (C) 2017  Martin Huus Bjerge, Rope Robotics ApS, Denmark

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL "Rope Robotics ApS" BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of "Rope Robotics ApS" shall not be used 
in advertising or otherwise to promote the sale, use or other dealings in this Software 
without prior written authorization from "Rope Robotics ApS".

'''

__author__ = "Martin Huus Bjerge"
__copyright__ = "Copyright 2017, Rope Robotics ApS, Denmark"
__license__ = "MIT License"

import URBasic
import numpy as np
import time
from URBasic import kinematic
from URBasic.kinematic import Invkine_manip, Tran_Mat2Pose, Pose2Tran_Mat

host = '172.31.1.115'  # E.g. a Universal Robot offline simulator, please adjust to match your IP
acc = 0.9
vel = 0.3

robotModle = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=robotModle)
robot.reset_error()


def moveDirectionMat(startPosition, movementMat):
    rot = np.array([[0.7071, -0.7071, 0, 0],
                    [0.7071, 0.7071, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    movementMat = np.matmul(rot, movementMat)
    newPosition = startPosition[:, -1] + movementMat[:, -1] - np.array([0, 0, 0, 1])
    newPosition = np.column_stack((startPosition[:, 0:3], newPosition))
    robot.movej(pose=kinematic.Tran_Mat2Pose(startPosition), a=acc, v=vel)
    robot.movej(pose=kinematic.Tran_Mat2Pose(newPosition), a=acc, v=vel)
    robot.close()


def moveDirectionVec(startPosition, movementVec):
    '''
    Function for moving the ur10 to a desired position via a 3 dimensional vector in meters.
    The function requires a start position,a vector, and a velocity and acceleration
    '''
    rot = np.array([[0.7071, -0.7071, 0],
                    [0.7071, 0.7071, 0],
                    [0, 0, 1]])
    movementVec = np.matmul(rot, movementVec)
    newPosition = startPosition[:, -1] + np.append(movementVec, 0)
    newPosition = np.column_stack((startPosition[:, 0:3], newPosition))
    robot.movep(pose=kinematic.Tran_Mat2Pose(newPosition), a=acc, v=vel)
    robot.close()
    return newPosition


def moveservoj(startPosition, movementVec, gain):
    '''
    Function for move by servoj for the ur10 to a desired position with no rotation of EE.
    parameters:
    startPosition   :start position as pose
    momentVec       :dimensional vector for movement with regards to x,y,z.
    gain(optional)  :how fast the robot should adjust, ranging from 100-2000

    NOTE: giving the function a large distance will result in the robot moving extremely quickly!
    '''
    rot = np.array([[0.7071, -0.7071, 0],
                    [0.7071, 0.7071, 0],
                    [0, 0, 1]])
    startPosition = Pose2Tran_Mat(startPosition)
    jointValues = robot.get_actual_joint_positions()
    jointValues = [-1.4, -1, -2.5, -0.8, -0.2, 1.3]
    movementVec = np.matmul(rot, movementVec)
    newPosition = startPosition[:, -1] + np.append(movementVec, 0)
    newPosition = np.column_stack((startPosition[:, 0:3], newPosition))
    newPose = Tran_Mat2Pose(newPosition)
    inverseKinematics = Invkine_manip(newPose, jointValues)
    '''
    print(f"inverse solution{inverseKinematics}")
    print(startPosition)
    print(newPosition)
    '''
    robot.servoj(inverseKinematics, t=0.5, lookahead_time=0.05, gain=100, wait=True)


def ExampleurScript():
    '''
    This is a small example of how to connect to a Universal Robots robot and use a few simple script commands.
    The scrips available is in general all the scrips from the universal robot script manual,
    and the implementation is intended to follow the Universal Robots manual as much as possible.

    This script can be run connected to a Universal Robot robot (tested at a UR5) or a Universal Robot offline simulator.
    See this example in how to setup an offline simulator:
    https://www.universal-robots.com/download/?option=26266#section16597
    '''

    # set the robot in a initial start position
    ur10Pose = np.array([[0, -0.7071, -0.7071, -0.3],
                         [0, 0.7071, -0.7071, - 0.3],
                         [1, 0, 0, 0.300],
                         [0, 0, 0, 1]])
    robot.movej(pose=kinematic.Tran_Mat2Pose(ur10Pose), a=acc, v=vel)

    # create a dummy vector for the robot to move
    movementVector = np.array([0, 0, 0.01])

    # get the current pose and move until reaches it's goal of 1 meter in up in z axis.
    pose = robot.get_actual_tcp_pose()
    while pose[2] < 1:
        moveservoj(pose, movementVector)
        pose = robot.get_actual_tcp_pose()
        print(pose)


if __name__ == '__main__':
    ExampleurScript()
