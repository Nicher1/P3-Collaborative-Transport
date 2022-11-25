import numpy as np
import roboticstoolbox as rtb

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

    ur10Pose = np.array([[0, -0.7071, -0.7071, 0],
                          [0, 0.7071, -0.7071, 0],
                          [1, 0, 0, 0.300],
                          [0, 0, 0, 1]])
    movementVector = np.array([0, 0, 0.4])
    moveDirection(ur10Pose, movementVector)


if __name__ == '__main__':
    ExampleurScript()
