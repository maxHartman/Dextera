import time
from dof import Sensor, Encoder, AnalogInput, GearMotor,ServoMotor, ServoDOF, MotorPIDDOF
from dof import Gripper
import pigpio
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)


GRIPPER_L_PIN_A = 0
GRIPPER_L_PIN_B = 22
GRIPPER_R_PIN_A = 17
GRIPPER_R_PIN_B = 5

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
wristRotate = MotorPIDDOF(pi, wristRotateMotor, wristAngleSensor, 1, kp=-.001, ki=-.000, kd=-.0000, MIN=-0, MAX=1000)
elevator = MotorPIDDOF(pi, elevatorMotor, encoder, 0, kp = 0.05, ki=0, kd=0, MIN=-9000, MAX=0)
degree_val = 0 
thous_range = 2500 - (500)
deg_range = 90 +90
new_value = (degree_val - (-90)) * thous_range / deg_range + 500
print(int(new_value))
wristPan.set_position(0)
elevatorMotor.set_power(0)
gripper_1 = Gripper(gripperMotorR)
gripper_2 = Gripper(gripperMotorL)

wristRotate.set_position(300)
wristPan.set_position(0)
gripper_2.open()
gripper_2.close()
elevator.set_position(-5000)
wristPan.set_position(-15)
time.sleep(5)
elevator.disable_pid()
elevatorMotor.set_power(0.9)
time.sleep(6)
elevatorMotor.set_power(0)
#elevator.set_position(1500)
wristPan.set_position(15)
#wristRotate.set_position(800)
time.sleep(5)
wristPan.set_position(0)

time.sleep(10)
while True:
    pass

testMotors = 0
while testMotors:
    #test motor classes
#    gripperMotorR.set_power(.5)
#    gripperMotorL.set_power(.5)
   # wristRotateMotor.set_power(0.9)
    #elevatorMotor.set_power(0.9)
    time.sleep(10)
#    #test dofs
    wristPan.set_position(60)
#    wristRotate.set_position(200)
#    time.sleep(3.5)
#    gripperMotorR.set_power(-.5)
#    gripperMotorL.set_power(-.5)
#    wristRotateMotor.set_power(0)
    #elevatorMotor.set_power(0.9)
    wristPan.set_position(-60)
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
    
 
    #wristRotate.set_position(200)
    elevator.set_position(500)
    #print(wristAngleSensor.get_value())
    time.sleep(10)
    print(elevator.sensor.get_value())

    print("switching")
    #wristRotate.set_position(800)
    elevator.set_position(-500)
    time.sleep(10)
    #print(elevator.sensor.get_value())

    print("switching")
    

