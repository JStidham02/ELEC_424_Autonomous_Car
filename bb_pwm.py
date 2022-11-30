class BB_PWM():

    def set_duty_cycle(self, pin, cycle):
        """
        This method sets the duty cycle

        pin must be either "P9_14 or P9_16"

        cycle should be a numeric type between 5 and 10

        returns true upon success
        returns false on failure
        """

        if cycle < 5 or cycle > 10:
            return False   
        else:

            period = int((cycle / 100.0) * 20000000)

            if pin == "P9_14":
                with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
                    filetowrite.write(period.__str__())
                return True
            elif pin == "P9_16":
                with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
                    filetowrite.write(period.__str__())
                return True
            else:
                return False


    def default_vals(self, pin):
        """
        This method returns the specified pins PWM to default values (stopped or centered)
        
        pin must be either "P9_14" or "P9_16"
        """

        if pin == "P9_14": # motor
            with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
                filetowrite.write("1600000")
            return True
        elif pin == "P9_16": # steering
            with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
                filetowrite.write("1500000")
            return True
        else:
            return False
