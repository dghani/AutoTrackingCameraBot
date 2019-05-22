# BEE 526 A
# 3/11/2019

# Python 2.7 script

from subprocess import check_call
# possibly use os.system() instead of subprocess.check_call() for faster performance.

# *** WARNING: This code is completely insecure!!!!! ***
# Never pass untrusted input to the CameraPanTilt class.


# Assumes pi-blaster is running already
# Assumes pi-blaster's PWM base rate is 100Hz (the default). 
class CameraPanTilt:    
    def __init__(self):
        self.HORIZ_GPIO_PIN = 17
        self.VERT_GPIO_PIN  = 27

        # All of these PWM constants below must be floats
        
        self.MIN_HORIZ_PWM = 0.24
        self.MAX_HORIZ_PWM = 0.075

        self.MIN_HORIZ_ANGLE = -90.0
        self.MAX_HORIZ_ANGLE =  90.0
        
        self.MIN_VERT_PWM = 0.120
        self.MAX_VERT_PWM = 0.175

        self.MIN_VERT_ANGLE = -20.0
        self.MAX_VERT_ANGLE =  45.0
        
        self.INITIAL_HORIZ_PWM = 0.1575
        self.INITIAL_VERT_PWM  = 0.14
        
        self.last_horiz_pwm = None
        self.last_vert_pwm = None
        
        self.NUDGE_RATIO = 0.05 # 0 < NUDGE_RATIO < 1 
        
        # {} #1: the GPIO pin number
        # {} #2: the PWM ratio
        self.COMMAND_STRING = 'echo "{}={}" > /dev/pi-blaster'
        
        #print("Initializing CameraPanTilt.")
        #print("Moving camera to default position.")
        self.go_to_initial_position()
    
    def go_to_initial_position(self):
        self.horiz_pwm(self.INITIAL_HORIZ_PWM)
        self.vert_pwm(self.INITIAL_VERT_PWM)
    
    def linear_interp(self, x, x1, y1, x2, y2):
        # line defined by two points: (x1, y1) and (x2, y2)
        y = ((y2 - y1)/float(x2 - x1))*(x - x1) + y1
        return y
    
    def horiz_angle_to_pwm(self, angle):
        
        x1 = self.MIN_HORIZ_ANGLE
        y1 = self.MIN_HORIZ_PWM
        
        x2 = self.MAX_HORIZ_ANGLE
        y2 = self.MAX_HORIZ_PWM
        
        x = angle
    
        pwm_ratio = self.linear_interp(x, x1, y1, x2, y2) 
        
        return pwm_ratio
        
    def vert_angle_to_pwm(self, angle):
        
        x1 = self.MIN_VERT_ANGLE
        y1 = self.MIN_VERT_PWM
        
        x2 = self.MAX_VERT_ANGLE
        y2 = self.MAX_VERT_PWM
        
        x = angle
    
        pwm_ratio = self.linear_interp(x, x1, y1, x2, y2) 
        
        return pwm_ratio
        
    def horiz_percent_to_pwm(self, percent):
        percent = float(percent)
        
    
    def horiz_pwm(self, pwm_ratio):
        ratio = self.clamp(pwm_ratio, self.MIN_HORIZ_PWM, self.MAX_HORIZ_PWM)
        command = self.COMMAND_STRING.format(self.HORIZ_GPIO_PIN, ratio)
        check_call([command], shell=True)
        self.last_horiz_pwm = ratio
        
    def horiz_angle(self, angle):
        pwm_ratio = self.horiz_angle_to_pwm(angle)
        self.horiz_pwm(pwm_ratio)
        
    
        
    def vert_pwm(self, pwm_ratio):
        ratio = self.clamp(pwm_ratio, self.MIN_VERT_PWM, self.MAX_VERT_PWM)
        command = self.COMMAND_STRING.format(self.VERT_GPIO_PIN, ratio)
        check_call([command], shell=True)
        self.last_vert_pwm = ratio
        
    def vert_angle(self, angle):
        pwm_ratio = self.vert_angle_to_pwm(angle)
        self.vert_pwm(pwm_ratio)
        
    def nudge_left(self):
        nudge = self.NUDGE_RATIO * abs(self.MIN_HORIZ_PWM - self.MAX_HORIZ_PWM)
        self.horiz_pwm(self.last_horiz_pwm + nudge)
        
    def nudge_right(self):
        nudge = self.NUDGE_RATIO * abs(self.MIN_HORIZ_PWM - self.MAX_HORIZ_PWM)
        self.horiz_pwm(self.last_horiz_pwm - nudge)
        
    def nudge_down(self):
        nudge = self.NUDGE_RATIO * abs(self.MIN_VERT_PWM - self.MAX_VERT_PWM)
        self.vert_pwm(self.last_vert_pwm - nudge)
        
    def nudge_up(self):
        nudge = self.NUDGE_RATIO * abs(self.MIN_VERT_PWM - self.MAX_VERT_PWM)
        self.vert_pwm(self.last_vert_pwm + nudge)
        
    # Thanks to StackOverflow
    # https://stackoverflow.com/a/5996949
    def clamp(self, n, minn, maxn):
        if maxn < minn: # swap if bounds were reversed
            minn, maxn = maxn, minn
        # Clamp
        if n < minn:
            return minn
        elif n > maxn:
            return maxn
        else:
            return n
    
#def test_main():
#    print("test_main() started.")
#    cam = CameraPanTilt()
    
# test main
#if __name__ == "__main__":
#    print("CameraPanTilt: beginning test_main().")
#    test_main()