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
from URBasic import kinematic
import time

host = '172.31.1.115'  # E.g. a Universal Robot offline simulator, please adjust to match your IP
acc = 0.9
vel = 0.2


def ExampleurScript():
    '''
    This is a small example of how to connect to a Universal Robots robot and use a few simple script commands.
    The scrips available is in general all the scrips from the universal robot script manual,
    and the implementation is intended to follow the Universal Robots manual as much as possible.

    This script can be run connected to a Universal Robot robot (tested at a UR5) or a Universal Robot offline simulator.
    See this example in how to setup an offline simulator:
    https://www.universal-robots.com/download/?option=26266#section16597
    '''
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
        robot.servoj(pose,t=0.008,lookahead_time=0.1, gain=100)
        #robot.close()
        return newPosition

    def interpolation(pose,vector):
        while pose[2,3]<0.8:
              print("dsev")
              pose = moveDirectionVec(pose, vector)


    ur10Pose = np.array([[0, -0.7071, -0.7071, -0.3],
                         [0, 0.7071, -0.7071, - 0.3],
                         [1, 0, 0, 0.300],
                         [0, 0, 0, 1]])
    pose = np.array([[0, -0.7071, -0.7071, -0.3],
                         [0, 0.7071, -0.7071, - 0.3],
                         [1, 0, 0, 0.600],
                         [0, 0, 0, 1]])
    movementVector = np.array([0, 0, 0.01])
    movementVector1 = np.array([0, 0.6, 0])
    robot.movej(pose=kinematic.Tran_Mat2Pose(ur10Pose), a=acc, v=vel)
    q = robot.get_inverse_kin(pose)
    print(q)
    interpolation(ur10Pose,movementVector)

if __name__ == '__main__':
    ExampleurScript()
