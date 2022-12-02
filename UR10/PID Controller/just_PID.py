import time

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

# Main

constants_y = [1,0.002,0.01]
constants_xz = [1, 1, 1]
PIDy = PID(Kp=constants_y[0], Ki=constants_y[1], Kd=constants_y[2], lim_max=300, lim_min=0)
PIDxz = PID(Kp=constants_xz[0], Ki=constants_xz[1], Kd=constants_xz[2], lim_max=100, lim_min=0) # A position of max 100 mm will give a velocity of 125 mm/s

while 1:
    if SP[1] > 0:
        VP = dryve.getPosition()  # Needs to be converted to UDP
        velocity = PIDy.update(feedback=VP, target=SP[1])
        velocity = int(round(velocity))
        dryve.targetVelocity(velocity) # Needs to be converted to UDP

        if velocity < 1:
            dryve.targetVelocity(0)  # Needs to be converted to UDP

    if SP[0] > 0:
        VP = 1  # Get position  + # Needs to be converted to UDP
        position = PIDxz.update(feedback=VP, target=SP[0])
        position = int(round(position))
        # DO THE THING :)   + # Needs to be converted to UDP

        if position < 1:
            pass
            # Set position to the current  + # Needs to be converted to UDP