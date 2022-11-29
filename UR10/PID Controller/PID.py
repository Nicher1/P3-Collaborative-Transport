from cmath import inf
import time
import createdryverail as dryve
import matplotlib as plt


class PID:
    # https://electronics.stackexchange.com/questions/629032/pid-controller-implementation-in-python

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
        self.set_limits(0, inf, -inf, inf)

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

        # Uhhhh (maybe change delta_time depending on how long our code is?) (Perhaps calculate delta time using last_time and current_time)
        delta_time = 0.001
        if delta_time == 0:
            return self.last_output

        # Calculates P, I and D
        self.Pterm = self.Kp * error
        self.Iterm += (error + self.last_error) * 0.5 * self.Ki * delta_time
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
        print(f"P: {self.Pterm}, I: {self.Iterm}, f: {feedback}")

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


class interpritor:

    def __init__(self, max_speed: float, error: float, Kp: float, Ki: float, Kd: float) -> None:
        self.max_speed = max_speed
        self.error = error
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        P = self.Kp * self.error
        I = (self.error + 0) * 0.5 * self.Ki * 0.001
        D = self.Kd * (self.error - 0) / (0.001)

        self.PID_weight = P + I + D

    def interpriate(self, PID_result: float) -> None:
        velocity = self.max_speed * (PID_result / self.PID_weight)
        return velocity


def plotter(PID, time):
    x = [0, 5, 10, 15]
    y = [200, 200, 200, 200]

    plt.plot(x, y, label="SP")
    plt.plot(time, PID, label="VP")

    plt.legend()
    plt.show()


# Main
run = True
constants_y = [1, 0, 0]
SP = [0, 201, 0]
dryve.dryveInit()
dryve.targetPosition(1)
PID_holder = deque()
time_holder = deque()

PIDy = PID(Kp=constants_y[0], Ki=constants_y[1], Kd=constants_y[2])
interpritor_y = interpritor(max_speed=1, error=200, Kp=constants_y[0], Ki=constants_y[1], Kd=constants_y[2])

while run == True:
    if xyz_vector[1] > 0:
        VP = dryve.getPosition()
        print(VP)
        PID_result = PIDy.update(feedback=VP, target=SP[1])
        PID_holder.append(PID_result)
        time_holder.append(time.time())
        velocity = interpriate(PID_result=PID_result)
        dryve.targetVelocity(velocity)
        if VP == SP[2]:
            run = False
plotter(PID=PID_holder, time=time_holder)
