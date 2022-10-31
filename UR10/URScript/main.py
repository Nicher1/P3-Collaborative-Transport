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
from URBasic import kinematic
import time
import numpy

host = '172.31.1.115'   #E.g. a Universal Robot offline simulator, please adjust to match your IP
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
    robot = URBasic.urScriptExt.UrScriptExt(host=host,robotModel=robotModle)
    robot.reset_error()

    rotasjon = numpy.array([[1, 0, 0, -0.6],
                            [0, 1, 0, -0.7],
                            [0, 0, 1, 0.400],
                            [0, 0, 0, 1]])

    # Units for pose is meters and radians: Pose = (X, Y, Z, rot, rot, rot)
    #robot.movej(pose=kinematic.Tran_Mat2Pose(rotasjon), a=acc, v=vel)
    robot.movej(pose=kinematic.Tran_Mat2Pose(rotasjon), a=acc, v=vel)


    robot.close()


if __name__ == '__main__':
    ExampleurScript()
