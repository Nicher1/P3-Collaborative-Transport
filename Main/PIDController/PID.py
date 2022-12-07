from cmath import inf
import time
from time import perf_counter
import createdryverail as dryve
import numpy as np
import matplotlib.pyplot as plt

# UR10 Setup

import URBasic
import time
import numpy as np
from URBasic import kinematic
from URBasic.kinematic import Invkine_manip, Tran_Mat2Pose, Pose2Tran_Mat

host = '172.31.1.115'  # E.g. a Universal Robot offline simulator, please adjust to match your IP
acc = 0.9
vel = 0.9


class PID:

    def __init__(self, Kp: float, Ki: float, Kd: float, lim_min: float, lim_max: float) -> None:  # , tau: float
        # Defines the datatype for each variable and gives them a starting value of none

        # Creates the weight variables
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.lim_max = lim_max
        self.lim_min = lim_min

        # Time constraint?
        # self.tau = tau

        # Sets the initial value of P, I and D to zero
        self.Dterm = 0
        self.Iterm = 0
        self.last_error = 0

        # Saved the current time as the start parameter
        self.last_time = time.time()

        # Is it necessary? It is never used
        self.last_feedback = 0

        self.last_output = 0

        # Sets the thresholding
        self.set_limits(lim_min, lim_max, -inf, inf)

    def set_limits(self, min: float, max: float, min_int: float, max_int: float) -> None:

        # Creates the variable for thresholding the output
        self.max = max
        self.min = min

        # Creates the variables for thresholding the integration
        self.max_int = max_int
        self.min_int = min_int

    def update(self, feedback: float, target: float) -> float:
        # Calculates the error
        error = target - feedback

        # Sets the current time
        current_time = time.time()

        # Calculates P, I and D (In deskrete time)
        self.Pterm = self.Kp * error
        self.Iterm += (error + self.last_error) * 0.5 * self.Ki * (current_time - self.last_time)
        self.Dterm = self.Kd * (error - self.last_error) / (current_time - self.last_time)

        # Thresholds the integration
        if self.Iterm > self.max_int:
            self.Iterm = self.max_int
        elif self.Iterm < self.min_int:
            self.Iterm = self.min_int

        # Saves current variables for later use
        self.last_time = current_time
        self.last_error = error
        self.last_feedback = feedback
        # feedback = VP

        # Prints the calculated P, I and VP
        # print(f"P: {self.Pterm}, I: {self.Iterm}, f: {feedback}")

        # Does the PID equation (MV)
        output = self.Pterm + self.Iterm + self.Dterm

        # Thresholds the output
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max

        # Saves the current output as the most recent output (used for next iteration)
        self.last_output = output
        self.last_time = time.time()

        # Returns the output
        return output


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

    def inverseTranMat(self, coordinate):
        currentPose = self.robot.get_actual_tcp_pose()
        tranMat = kinematic.Pose2Tran_Mat(currentPose)
        tranMat = np.linalg.inv(tranMat)
        return int(round(tranMat[coordinate, -1] * 1000))


# Main Setup (Can be removed)
run = True
SP = [-500, 0, 0]  # For xz the coordinate needs to be the tform of starting position + movement in desired direction
starting_time = time.time()
plt.axhline(y=-500, color='orange', linestyle='-')
plt.xlabel("Time")
plt.title("PID Controller")
x = list()
y = list()

#dryve.dryveInit()  # (Commented out since I am testing the UR10)
# dryve.targetPosition(300)

# Initilize robot
ur10 = Robot()
ur10.setup()

# Main Setup (This needs to remain in main)
constants_y = [1, 0.002, 0.01]  # [1,0.002,0.01]
constants_xz = [1, 0.002, 0.04] # [1,0.002,0.04]
PIDy = PID(Kp=constants_y[0], Ki=constants_y[1], Kd=constants_y[2], lim_max=300, lim_min=-300)
PIDx = PID(Kp=constants_xz[0], Ki=constants_xz[1], Kd=constants_xz[2], lim_max=100, lim_min=-100)
PIDz = PID(Kp=constants_xz[0], Ki=constants_xz[1], Kd=constants_xz[2], lim_max=100,
           lim_min=-100)  # A position of max 100 mm will give a velocity of 125 mm/s

positionx = 0
positionz = 0

time_constant = perf_counter()
while run == True:
    if SP[1] != 0:

        PV = dryve.getPosition()  # Needs to be converted to UDP
        time.sleep(0.0001)
        velocity = PIDy.update(feedback=PV, target=SP[1])
        velocity = int(round(velocity))
        dryve.targetVelocity(velocity)
        x.append(perf_counter() - time_constant)
        y.append(PV)

        if velocity == 0:
            run = False
            dryve.targetVelocity(0)

    if SP[0] != 0:
        xyz_list = ur10.robot.get_actual_tcp_pose()  # [x,y,z,rotx,roty,rotz]
        VPx = xyz_list[0] * 1000
        positionx = PIDx.update(feedback=VPx, target=SP[0])
        positionx = int(round(positionx))

    if SP[2] != 0:
        xyz_list = ur10.robot.get_actual_tcp_pose()  # [x,y,z,rotx,roty,rotz]
        VPz = xyz_list[2] * 1000
        positionz = PIDz.update(feedback=VPz, target=SP[2])
        positionz = int(round(positionz))

    ur10.moveRTC(positionx, positionz)
    time.sleep(0.001)
    print("I am at ", ur10.robot.get_actual_tcp_pose())
    x.append(perf_counter() - time_constant)
    y.append(VPx)


    if positionx == 0 and positionz == 0:
        run = False

# plotter(PID=PID_holder, time=time_holder)
time.sleep(1)
#ny_VP = dryve.getPosition()
#print(PV, " and ", ny_VP)
#xyz_check = robot.get_actual_tcp_pose()
# print(PV, " and ", xyz_check[0])
plt.plot(x, y)
plt.show()
