import time
from dof import Sensor, Encoder, AnalogInput, GearMotor,ServoMotor, ServoDOF, MotorPIDDOF
import pigpio
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)


GRIPPER_L_PIN_A = 0
GRIPPER_L_PIN_B = 22
GRIPPER_R_PIN_A = 5
GRIPPER_R_PIN_B = 17

ELEVATOR_MOTOR_PIN_A = 2
ELEVATOR_MOTOR_PIN_B = 3
ELEVATOR_ENCODER_PIN_A = 7
ELEVATOR_ENCODER_PIN_B = 1

WRIST_PAN_PIN = 18
WRIST_ROTATE_SENSOR_PIN = 0
WRIST_ROTATE_SERVO_PIN = 19
pi = pigpio.pi()
encoder = Encoder(pi, ELEVATOR_ENCODER_PIN_A, ELEVATOR_ENCODER_PIN_B)
wristAngleSensor = AnalogInput(pi)

gripperMotorR = GearMotor(pi, GRIPPER_L_PIN_A, GRIPPER_L_PIN_B)
gripperMotorL = GearMotor(pi, GRIPPER_R_PIN_A, GRIPPER_R_PIN_B)
wristRotateMotor = ServoMotor(pi, WRIST_ROTATE_SERVO_PIN)
elevatorMotor = GearMotor(pi, ELEVATOR_MOTOR_PIN_A, ELEVATOR_MOTOR_PIN_B)

wristPan = ServoDOF(pi, WRIST_PAN_PIN)
wristRotate = MotorPIDDOF(pi, wristRotateMotor, wristAngleSensor, 1, kp=-.001, ki=-.000, kd=-.0000)
elevator = MotorPIDDOF(pi, elevatorMotor, encoder, 0, kp = 0.05, ki=0, kd=0)

wristPan.set_position(45)
elevatorMotor.set_power(0)

testMotors = 1
while testMotors:
    #test motor classes
#    gripperMotorR.set_power(.5)
#    gripperMotorL.set_power(.5)
    wristRotateMotor.set_power(0.9)
    elevatorMotor.set_power(-0.9)
    time.sleep(10)
#    #test dofs
#    wristPan.set_position(45)
#    wristRotate.set_position(200)
#    time.sleep(3.5)
#    gripperMotorR.set_power(-.5)
#    gripperMotorL.set_power(-.5)
#    wristRotateMotor.set_power(0)
    elevatorMotor.set_power(-1)
    #wristPan.set_position(0)
    #wristRotateMotor.set_power(-0.5)
    #wristRotate.set_position(700)
    time.sleep(10)
    #print(wristRotate.get_counter())
    
testSensors = 0
while testSensors:
    print(wristAngleSensor.get_value())
    time.sleep(.001)
    
testDOFs = 1
while testDOFs:
    
 
    wristRotate.set_position(200)
   # elevator.set_position(1500)
    print(wristAngleSensor.get_value())
    time.sleep(10)
    #print(elevator.sensor.get_value())

    print("switching")
    wristRotate.set_position(800)
    #elevator.set_position(-1500)
    time.sleep(10)
    #print(elevator.sensor.get_value())

    print("switching")
    

