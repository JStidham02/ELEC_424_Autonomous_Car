class Error_PID_Controller():
    """
    This is an implementation of a PID Controller that operates on the error between a desired value and the current value
    """


    # desired value
    desired = 0.0
    
    # store gains
    p_gain = 0.0
    i_gain = 0.0
    d_gain = 0.0

    # to calculate derivative
    last_error = 0.0

    
    current_sample = 0.0
    derivative = 0.0
    integral = 0.0

    output = 0.0


    def set_target(self, target):
        """
        This method sets a new target value and resets the integral counter
        """
        self.desierd = target
        self.integral = 0.0
        return True


    def set_p_gain(self, gain):
        """
        This method sets the proportional gain
        """
        self.p_gain = gain
        return True

    def get_p_gain(self):
        """
        This method gets the proportional gain
        """
        return self.p_gain


    def set_i_gain(self, gain):
        """
        This method sets the integral gain
        """
        self.i_gain = gain
        return True

    def get_i_gain(self):
        """
        This method gets the integral gain
        """
        return self.i_gain


    def set_d_gain(self, gain):
        """
        This method sets the derivative gain
        """
        self.d_gain = gain
        return True

    def get_d_gain(self):
        """
        This method gets the derivative gain
        """
        return self.d_gain

    def get_error(self):

        return self.last_error

    def get_error_derivative(self):

        return self.derivative

    def get_error_integral(self):

        return self.integral

    def get_output_val(self):
        """
        This method returns the current PID output value
        """
        return self.output


    def update_pid(self, new_val):
        """
        This method updates the PID controller with a new sample and computes new values

        It returns the new controller output value, which can alse be retrieved from the get_output_val method
        """
        error = self.desierd - new_val
        self.integral = self.integral + error
        self.derivative = error - self.last_error
        self.last_error = error
        sum = 0
        sum = sum + self.integral * self.i_gain
        sum = sum + self.derivative * self.d_gain
        sum = sum + error* self.p_gain
        self.output = sum
        return self.output


    