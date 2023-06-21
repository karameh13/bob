import Jetson.GPIO as GPIO
import 

class Motor:
    def __init__(self, directionPin, pwmPin, minSpeed, maxSpeed):
        self.m_directionPin = directionPin
        self.m_pwmPin = pwmPin
        self.m_minSpeed = minSpeed
        self.m_maxSpeed = maxSpeed
        self.m_speed = 0.0
        self.m_direction = 0

        # Pin configurations for JetsonGPIO
        GPIO.setmode(GPIO.BOARD)  # GPIO.BCM or GPIO.BOARD
        GPIO.setup(32, GPIO.OUT, initial=GPIO.HIGH)  # Left motor
        GPIO.setup(33, GPIO.OUT, initial=GPIO.HIGH)  # Right motor
        GPIO.setup(31, GPIO.OUT, initial=GPIO.HIGH)  # Left direction
        GPIO.setup(35, GPIO.OUT, initial=GPIO.HIGH)  # Right direction

    # --- Pwm pin ---
    def getPwmPin(self):
        return self.m_pwmPin
    # --- End of Pwm pin ---

    # --- Wheel direction ---
    def setDirection(self, speed):
        if speed >= 0:
            self.m_direction = True
        elif speed < 0:
            self.m_direction = False

    def getDirection(self):
        return self.m_direction

    def getDirectionPin(self):
        return self.m_directionPin
    # --- End of wheel direction ---

    # --- Control of motor actions ---
    def setSpeed(self, speed):
        if speed <= self.m_minSpeed:
            self.m_speed = self.m_minSpeed
        elif speed >= self.m_maxSpeed:
            self.m_speed = self.m_maxSpeed
        else:
            self.m_speed = speed

    def getSpeed(self):
        return self.m_speed

    def setMinSpeed(self, minSpeed):
        self.m_minSpeed = minSpeed

    def setMaxSpeed(self, maxSpeed):
        self.m_maxSpeed = maxSpeed

    def stop(self):
        self.m_speed = 0.0
    # --- End of control of motor actions ---

