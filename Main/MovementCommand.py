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
import time
import numpy as np
from URBasic import kinematic
from URBasic.kinematic import Invkine_manip, Tran_Mat2Pose, Pose2Tran_Mat

host = '172.31.1.115'  # E.g. a Universal Robot offline simulator, please adjust to match your IP
acc = 0.9
vel = 0.9


class Robot:

    def __init__(self):
        self.robotMod = URBasic.robotModel.RobotModel()
        self.robot = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=self.robotMod)
        self.robot.reset_error()

    def setup(self):
        ur10Pose = np.array([[-0.7071, 0, -0.7071, -0.3],
                             [0.7071, 0, -0.7071, - 0.3],
                             [0, -1, 0, 0.3],
                             [0, 0, 0, 1]])
        self.robot.movej(pose=kinematic.Tran_Mat2Pose(ur10Pose), a=acc, v=vel)
        time.sleep(1)

    def moveRTC(self, x, z):
        '''
        Real time movement given an x and z vector.
        '''
        rot = np.array([[0.7071, -0.7071, 0],
                        [0.7071, 0.7071, 0],
                        [0, 0, 1]])
        movementVec = np.array([x / 1000, 0, z / 1000])
        movementVec = np.matmul(rot, movementVec)
        currentPose = self.robot.get_actual_tcp_pose()
        currentPose[0:3] = currentPose[0:3] + movementVec
        self.robot.set_realtime_pose(currentPose)

    def getCurrentTranMat(self, coordinate):
        rotation = np.array([[0.7071, 0.7071, 0, 0],
                        [-0.7071, 0.7071, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        currentPose = self.robot.get_actual_tcp_pose()
        tranMat = kinematic.Pose2Tran_Mat(currentPose)
        tranMat = np.matmul(rotation, tranMat)
        print(tranMat)
        return int(round(tranMat[coordinate, -1] * 1000))


if __name__ == "__main__":
    UR10 = Robot()
    UR10.setup()
    UR10.moveRTC(-400, 100)
    time.sleep(1)
    poseX = UR10.getCurrentTranMat(0)
    poseY = UR10.getCurrentTranMat(1)
    poseZ = UR10.getCurrentTranMat(2)
    print(poseX, poseY, poseZ)
    time.sleep(1)
