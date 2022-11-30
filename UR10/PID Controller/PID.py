from cmath import inf
import time
import createdryverail as dryve
import matplotlib.pyplot as plt
from collections import deque


class PID:

    def __init__(self, Kp: float, Ki: float, Kd: float) -> None:  # , tau: float
        # Defines the datatype for each variable and gives them a starting value of none

        # Creates the weight variables
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

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
        self.set_limits(0, 300, -inf, inf)

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

        # Calculates P, I and D
        self.Pterm = self.Kp * error
        self.Iterm += (error + self.last_error) * 0.5 * self.Ki * (current_time - self.last_time)
        # Calculates the integral in diskret time

        # Maybe use error instead of feedback? Since the error might change over time
        self.Dterm = self.Kd * (error - self.last_error) / (current_time - self.last_time)

        # self.Dterm = (-2 * self.Kd * (feedback - self.last_feedback) + (2 * self.tau - delta_time) * self.Dterm / (2 * self.tau + delta_time))
        # ???

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
        #print(f"P: {self.Pterm}, I: {self.Iterm}, f: {feedback}")

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

    def interpriate(self, max_speed: float, previous_PID: float, current_PID: float):
        velocity = current_PID
        print(velocity)

        return velocity


def plotter(PID, time):
    x = [0, 0.25, 0.75, 1]
    y = [200, 200, 200, 200]

    plt.plot(x, y, label="SP")
    plt.plot(time, PID, label="VP")

    plt.legend()
    plt.show()


# Main
run = True
constants_y = [1, 0.01, 0.1]
SP = [0, 201, 0]
dryve.dryveInit()
PID_holder = deque()
time_holder = deque()
starting_time = time.time()
PIDy = PID(Kp=constants_y[0], Ki=constants_y[1], Kd=constants_y[2])


while run == True:
    if SP[1] > 0:
        VP = dryve.getPosition()
        PID_result = PIDy.update(feedback=VP, target=SP[1])
        #time.sleep(0.0000000001)
        velocity = PIDy.interpriate(max_speed=300, current_PID=PID_result, previous_PID = previous_PID)
        PID_holder.append(VP)
        time_holder.append(time.time()-starting_time)
        starting_time = time.time()
        velocity = int(round(velocity))
        dryve.targetVelocity(velocity)

        if velocity < 1:
            run = False
            dryve.targetVelocity(0)

#plotter(PID=PID_holder, time=time_holder)
time.sleep(1)
ny_VP = dryve.getPosition()
print(VP, " and ", ny_VP)