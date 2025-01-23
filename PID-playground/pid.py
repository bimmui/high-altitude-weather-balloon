"""PID Controller class based on Philip Salmony's C implementation"""


class PIDController:
    def __init__(self, Kp, Ki, Kd, tau, limMin, limMax, limMinInt, limMaxInt, T):
        """
        Initialize the PID controller with the given parameters.

        :param Kp: Proportional gain
        :param Ki: Integral gain
        :param Kd: Derivative gain
        :param tau: Derivative low-pass filter time constant
        :param limMin: Minimum output limit
        :param limMax: Maximum output limit
        :param limMinInt: Minimum integrator limit
        :param limMaxInt: Maximum integrator limit
        :param T: Sample time (in seconds)
        """
        # controller gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # derivative low-pass filter time constant
        self.tau = tau

        # output limits
        self.limMin = limMin
        self.limMax = limMax

        # integrator limits
        self.limMinInt = limMinInt
        self.limMaxInt = limMaxInt

        # sample time
        self.T = T

        # controller "memory"
        self.integrator = 0.0
        self.prevError = 0.0
        self.differentiator = 0.0
        self.prevMeasurement = 0.0

        # controller output
        self.out = 0.0

    def update(self, setpoint, measurement):
        """
        Update the PID controller.

        :param setpoint: Desired target value
        :param measurement: Current measured value
        :return: Control output
        """
        # error signal
        error = setpoint - measurement

        # proportional term
        proportional = self.Kp * error

        # integral term with anti-windup
        self.integrator += 0.5 * self.Ki * self.T * (error + self.prevError)

        # clamping integrator to limits
        if self.integrator > self.limMaxInt:
            self.integrator = self.limMaxInt
        elif self.integrator < self.limMinInt:
            self.integrator = self.limMinInt

        # derivative term (band-limited differentiator)
        self.differentiator = -(2.0 * self.Kd * (measurement - self.prevMeasurement)
                                + (2.0 * self.tau - self.T) * self.differentiator) / (2.0 * self.tau + self.T)

        # compute output and clamp to limits
        self.out = proportional + self.integrator + self.differentiator

        if self.out > self.limMax:
            self.out = self.limMax
        elif self.out < self.limMin:
            self.out = self.limMin

        # save error and measurement for the next update
        self.prevError = error
        self.prevMeasurement = measurement

        return self.out
