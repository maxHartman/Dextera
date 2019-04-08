import time
import pigpio
import RPi.GPIO as GPIO

# dof types
# pan servo 
# rotate cont. rotation servo
# elevator controller w/ PID

# gripper class - just open and close
# motor class - can set power, assign pins, limit motor output values (-1 to 1)
# sensor class - can get sensor value, assign pins

# dof class - has position, assigns position limits, can set position or velocity


# Motor  ---
class Motor(object):

    def __init__(self):
        pass

    # sets power between -1 and 1
    # needs to be overridden by subclasses
    def set_power(self, power):
        if abs(power) > 1.0:
            print("Error: motor assigned power outside -1 to 1")
            return 0
        return power 


class ServoMotor(Motor):
    
    CENTER_DUTY = 1500
    DUTY_AMPLITUDE = 500
    DUTY_MIN = CENTER_DUTY + DUTY_AMPLITUDE
    DUTY_MAX = CENTER_DUTY - DUTY_AMPLITUDE

    pi = None
    pin = None

    def __init__(self, pi, pin):
        super(ServoMotor, self).__init__()
        self.pi = pi
        self.pin = pin

    def set_power(self, power):
        power = super(ServoMotor, self).set_power(power)
        self.pi.set_servo_pulsewidth(self.pin, self.CENTER_DUTY + power * self.DUTY_AMPLITUDE)

class GearMotor(Motor):

    pi = None
    pin_a = None
    pin_b = None
    FREQUENCY = 1000
    FULL_ON = 1000000
    
    def __init__(self, pi, pin_a, pin_b):
        super(GearMotor, self).__init__()

        GPIO.setup(pin_a, GPIO.OUT)
        GPIO.setup(pin_b, GPIO.OUT)
        self.pin_a = GPIO.PWM(pin_a, self.FREQUENCY)
        self.pin_b = GPIO.PWM(pin_b, self.FREQUENCY)
        self.pin_a.start(0)
        self.pin_b.start(0)
        self.pi = pi

    def set_power(self, power):
        power = super(GearMotor, self).set_power(power)
        if power > 0:
            self.pin_a.ChangeDutyCycle(100 * power)
            self.pin_b.ChangeDutyCycle(0)
            pass
        if power < 0:
            self.pin_b.ChangeDutyCycle(-100*power)
            self.pin_a.ChangeDutyCycle(0)
            pass
        if power == 0:
            self.pin_a.ChangeDutyCycle(0)
            self.pin_b.ChangeDutyCycle(0)
            pass


# Sensors
class Sensor(object):

    def __init__(self):
        pass

    def get_value(self):
        pass
    
    def set_direction(self, direction):
        pass


class Encoder(Sensor):

    pi = None
    pin_a = None
    pin_b = None
    position = 0
    direction = 0
    
    def set_direction(self, direction):
        if not (self.direction == direction):
            print("changing direction")
        self.direction = direction
    """
        if self.pi.read(self.pin_b):
            self.position = self.position + 1
            direction = 1
        else:
            self.position = self.position - 1
            direction = -1
        if not (self.lastDirection == direction):
            print("switched direction")
        self.lastDirection = direction
"""
    def interrupt_callback(self, gpio, level, tick):
        # print("interrupted")
        if (self.direction == 0):
            print("Encoder no direction assigned")
        if (self.direction == 1):
            self.position = self.position + 1
        if (self.direction == -1):
            self.position = self.position - 1
            
    def reset_position(self):
        self.position = 0

    def __init__(self, pi, pin_a, pin_b):
        super(Encoder, self).__init__()
        self.pi = pi
        self.pin_a = pin_a
        self.pin_b = pin_b
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.pi.callback(self.pin_a, pigpio.RISING_EDGE, self.interrupt_callback)
        self.reset_position()

    def get_value(self):
        return self.position     


class AnalogInput(Sensor):

    # todo:allow reading from pin 1 (currently only reads pin 0)
    pi = None
    pin = None
    channel = None

    command = 0b11 << 6
    command |= (0 & 0x07) << 3

    def __init__(self, pi):
        super(AnalogInput, self).__init__()
        self.pi = pi
        self.channel = self.pi.spi_open(0, 1000000, 0)

    def set_direction(self, direction):
        pass
        #print("set direction called on analog sensor")

    def get_value(self):
        (count, resp) = self.pi.spi_xfer(self.channel, [self.command, 0x0, 0x0])
        result = (resp[0] & 0x01) << 9
        result |= (resp[1] & 0xFF) << 1
        result |= (resp[2] & 0x80) >> 7
        value = result & 0x3FF
        return value


class Gripper(object):

    motor = None
    close_power = -0.5
    open_power = 0.5
    sleep_time = 3

    def __init__(self, motor):
        self.motor = motor
        return

    def open(self):
        self.motor.set_power(self.open_power)
        time.sleep(self.sleep_time)  # TODO: maybe should make this non-blocking? idk if it matters
        self.motor.set_power(0)
        return

    def close(self):
        self.motor.set_power(self.close_power)
        time.sleep(self.sleep_time)  # TODO: maybe should make this non-blocking? idk if it matters
        self.motor.set_power(0)
        return


# DOFs
class DOF(object):

    def __init__(self):
        pass

    def set_position(self, position):
        pass

    def set_velocity(self, velocity):  # will effectively just set power
        pass

    def get_position(self):
        pass

class ServoDOF(DOF):
    
    FREQUENCY = 100    
    pi = None
    pin = None

    MIN_LIMIT = 500
    MAX_LIMIT = 2500

    MIN_DEGREE = -90
    MAX_DEGREE = 90

    def __init__(self, pi, pin):
        super(ServoDOF, self).__init__()
        self.pi = pi
        self.pin = pin
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        return

    def __convert_to_degrees(self, thousands_val):
        thous_range = self.MAX_LIMIT - self.MIN_LIMIT
        deg_range = self.MAX_DEGREE - self.MIN_DEGREE
        new_value = (thousands_val - self.MIN_LIMIT) * deg_range / thous_range + self.MIN_DEGREE
        return int(new_value)

    def __convert_to_thousands(self, degree_val):
        thous_range = self.MAX_LIMIT - self.MIN_LIMIT
        deg_range = self.MAX_DEGREE - self.MIN_DEGREE
        new_value = (degree_val - self.MIN_DEGREE) * thous_range / deg_range + self.MIN_LIMIT
        return int(new_value)

    def __in_limits(self, thous_position):
        return self.MIN_LIMIT <= thous_position <= self.MAX_LIMIT

    def __out_of_limits(self, thous_position):
        print('Position input exceeds limits ' + str(self.__convert_to_degrees(thous_position)))
        return

    def set_position(self, deg_position):
        thous_position = self.__convert_to_thousands(deg_position)
        if self.__in_limits(thous_position):
            self.pi.hardware_PWM(self.pin, self.FREQUENCY, int(thous_position*100))
            print(thous_position*50)
            print('pos in limits')
            return True
        self.__out_of_limits(thous_position)
        return False

    def get_position(self):
        print("can't get position of this servo")
        #thous_position = self.pi.get_servo_pulsewidth(self.pin)
        #return self.sensor.get_value()/360
        #return self.__convert_to_degrees(thous_position)

    def set_velocity(self, velocity):
        print ('NOT IMPLEMENTED')
        pass

    def stop(self):
        print ('NOT IMPLEMENTED')
        pass

pid_trigger_init = 0
class MotorPIDDOF(DOF):

    motor = None
    sensor = None
    integral = None
    last_error = None
    pid_enabled = 0
    last_time = None
    target = None
    continuous = 0
    kp = None
    ki = None
    kd = None
    pin_a = None
    pi = None

    V_POWER = .5
    R_POWER = 0.1
    FREQUENCY = 50  # Hz
    TRIGGER_OUT_PIN = 13
    TRIGGER_IN_PIN = 6
    counter = 0

    MIN = None
    MAX = None

    def __init__(self, pi, motor, sensor, continuous, kp, ki, kd, MIN, MAX):
        super(MotorPIDDOF, self).__init__()
        self.MIN = MIN
        self.MAX = MAX
        self.pi = pi
        self.motor = motor
        self.sensor = sensor
        self.continuous = continuous
        self.kp = kp
        self.ki = ki
        self.kd = kd
        global pid_trigger_init
        print(pid_trigger_init)
        if pid_trigger_init == 0:
            GPIO.setup(self.TRIGGER_OUT_PIN, GPIO.OUT)
            try:
                self.pin_a = GPIO.PWM(self.TRIGGER_OUT_PIN, self.FREQUENCY)
                print("starting pwm")
                self.pin_a.start(50)
                pid_trigger_init =1 
            except:
                print("caught error")
        self.pi.callback(self.TRIGGER_IN_PIN, pigpio.RISING_EDGE, self.interrupt_callback)
       
        return

    def stop(self):
        self.disable_pid()
        self.motor.set_power(0)
        # TODO: update location
        return

    def zero(self):
        self.motor.set_power(0.5)
        time.sleep(5)
        self.sensor.reset_position()
        self.set_position(-1000)
        time.sleep(5)	

    def set_velocity(self, direction):
        self.disable_pid()
        if self.continuous:
            self.motor.set_power(direction * self.R_POWER)
        self.motor.set_power(direction * self.V_POWER)

    def set_position(self, position):
        if (position < self.MAX) and (position > self.MIN):
            self.target = position
            self.enable_pid()
            return True
        else:
            print("position assigned out of range")
            return False

    def enable_pid(self):
        if self.pid_enabled == 0:
            self.pid_enabled = 1
            
    def disable_pid(self):
        self.pid_enabled = 0
        self.motor.set_power(0)
        self.last_time = None

    def get_counter(self):
        return self.counter

    def interrupt_callback(self, gpio, level, tick):
        self.update_pid()
        
    def update_pid(self):
        if self.pid_enabled == 0:
            return -1
        if (self.continuous == 1):
            rotationDistance = 1024
            sensor = self.sensor.get_value()
            
            errorCCWRot = (self.target - rotationDistance) - sensor
            errorNoRot = self.target - sensor
            errorCWRot = (self.target + rotationDistance) - sensor
            
            if (abs(errorCCWRot) < abs(errorNoRot)) and (abs(errorCCWRot) < abs(errorCWRot)):
                error = errorCCWRot
            if (abs(errorNoRot) < abs(errorCCWRot)) and (abs(errorNoRot) < abs(errorCWRot)):
                error = errorNoRot
            if (abs(errorCWRot) < abs(errorCCWRot)) and (abs(errorCWRot) < abs(errorNoRot)):
                error = errorCWRot

        else:
            error = self.target - self.sensor.get_value()
        
        if self.last_time is not None:
            d_t = time.clock() - self.last_time
            self.integral = self.integral + error+d_t
            derivative = (error - self.last_error)/d_t

            power = self.kp*error + self.ki*self.integral + self.kd*derivative
            self.motor.set_power(max(-1, min(1, power)))
            #print(power)
            if power > 0:
                self.sensor.set_direction(1)
            if power < 0:
                self.sensor.set_direction(-1)
        else:
            self.integral = 0

        self.last_error = error
        self.last_time = time.clock()
        self.counter = self.counter+1
        # temp for debugging thread calls
        # print(self.last_time)
        # if self.pid_enabled:
        #   self.s.enter(1/self.FREQUENCY, 0, self.update_pid)
        #   self.s.run()

    def get_position(self):  # TODO: IMPLEMENT
        return self.sensor.get_value()
