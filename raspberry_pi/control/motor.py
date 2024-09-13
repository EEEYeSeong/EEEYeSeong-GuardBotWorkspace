import RPi.GPIO as GPIO


"""
이 코드는 L298n 모터 드라이버 작동을 위한 코드입니다.
"""


class MotorGpio:
    GPIO_ENABLE_A = 1    # Left
    GPIO_ENABLE_B = 1    # Right
    GPIO_IN1 = 1         # Left - F
    GPIO_IN2 = 1         # Left - B
    GPIO_IN3 = 1         # Right - F
    GPIO_IN4 = 1         # Right - B

    PWM_HZ = 1000


class MotorController:
    def __init__(self, gpio_config: MotorGpio, gpio_mode = GPIO.BOARD):
        self.gpio_config = gpio_config

        GPIO.setmode(gpio_mode)
        GPIO.setup(self.gpio_config.GPIO_ENABLE_A, GPIO.OUT)
        GPIO.setup(self.gpio_config.GPIO_ENABLE_B, GPIO.OUT)
        GPIO.setup(self.gpio_config.GPIO_IN1, GPIO.OUT)
        GPIO.setup(self.gpio_config.GPIO_IN2, GPIO.OUT)
        GPIO.setup(self.gpio_config.GPIO_IN3, GPIO.OUT)
        GPIO.setup(self.gpio_config.GPIO_IN4, GPIO.OUT)

        self.enable_a_pwm = GPIO.PWM(self.gpio_config.GPIO_ENABLE_A, self.gpio_config.PWM_HZ)
        self.enable_b_pwm = GPIO.PWM(self.gpio_config.GPIO_ENABLE_B, self.gpio_config.PWM_HZ)
        self.enable_a_pwm.start(0)
        self.enable_b_pwm.start(0)
    
    def move_forward(self, speed):
        self.write(speed, speed, True, False, True, False)

    def move_backward(self, speed):
        self.write(speed, speed, False, True, False, True)

    def turn_left(self, speed):
        self.write(speed, speed, False, True, True, False)
    
    def turn_right(self, speed):
        self.write(speed, speed, True, False, False, True)

    def stop(self):
        self.write(0, 0, False, False, False, False)

    def write(self, ea: float, eb: float, i1: bool, i2: bool, i3: bool, i4: bool):
        # Internal func

        if not (0 <= ea <= 100) or not (0 <= eb <= 100):
            raise ValueError("The value of Enable A or Enable B must be greater than or equal to 0 and less than or equal to 100.")
        
        if ea == 0:
            self.enable_a_pwm.ChangeDutyCycle(0)
        else:
            self.enable_a_pwm.ChangeDutyCycle(ea)

        if eb == 0:
            self.enable_b_pwm.ChangeDutyCycle(0)
        else:
            self.enable_b_pwm.ChangeDutyCycle(eb)

        GPIO.output(self.gpio_config.GPIO_IN1, GPIO.OUT, i1)
        GPIO.output(self.gpio_config.GPIO_IN2, GPIO.OUT, i2)
        GPIO.output(self.gpio_config.GPIO_IN3, GPIO.OUT, i3)
        GPIO.output(self.gpio_config.GPIO_IN4, GPIO.OUT, i4)

    def cleanup(self):
        self.stop()
        self.enable_a_pwm.stop()
        self.enable_b_pwm.stop()
        
        GPIO.cleanup([
            self.gpio_config.GPIO_ENABLE_A,
            self.gpio_config.GPIO_ENABLE_B,
            self.gpio_config.GPIO_IN1,
            self.gpio_config.GPIO_IN2,
            self.gpio_config.GPIO_IN3,
            self.gpio_config.GPIO_IN4,
        ])
