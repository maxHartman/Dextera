import pigpio
import RPi.GPIO as GPIO
import re
import time

from dof import Gripper, Encoder, AnalogInput, GearMotor, ServoMotor, ServoDOF, MotorPIDDOF
from kinematics import FK, IK


class Arm:
    # TODO: MAKE THESE VARIABLES PRIVATE
    q = []  # joint angle positions (length 3) VERTICAL, ROTATE, PAN
    o_curr = []  # x, y, z in space

    GPIO.setmode(GPIO.BCM)

    VERTICAL_MOTOR = 0
    ROTATE_MOTOR = 1
    PAN_MOTOR = 2

    ROTATE_FACTOR = 1  # how much the servo rotates by when on continuous rotation

    GRIPPER_L_PIN_A = 0
    GRIPPER_L_PIN_B = 22
    GRIPPER_R_PIN_A = 17
    GRIPPER_R_PIN_B = 5

    ELEVATOR_MOTOR_PIN_A = 2
    ELEVATOR_MOTOR_PIN_B = 3
    ELEVATOR_ENCODER_PIN_A = 7
    ELEVATOR_ENCODER_PIN_B = 1

    NAME = 'jimmy'
    OFFSET = 0

    WRIST_PAN_PIN = 18
    WRIST_ROTATE_SENSOR_PIN = 0
    WRIST_ROTATE_SERVO_PIN = 19

    START_Q = [0, 500, 0]
    OFF_Q = [0, 400, 0]  # TODO: SET TO VALUES WE WANT

    MOVE_WORD = 'up'
    ROTATE_WORD = 'in'
    PAN_WORD = 'up'

    gripper_closed = [True, True]  # gripper 1 and gripper 2 status (closed is true)

    def __init__(self):
        pi = pigpio.pi()
        encoder = Encoder(pi, self.ELEVATOR_ENCODER_PIN_A, self.ELEVATOR_ENCODER_PIN_B)
        wrist_angle_sensor = AnalogInput(pi)
        gripper_motor_r = GearMotor(pi, self.GRIPPER_L_PIN_A, self.GRIPPER_L_PIN_B)
        gripper_motor_l = GearMotor(pi, self.GRIPPER_R_PIN_A, self.GRIPPER_R_PIN_B)
        wrist_rotate_motor = ServoMotor(pi, self.WRIST_ROTATE_SERVO_PIN)
        self.elevator_motor = GearMotor(pi, self.ELEVATOR_MOTOR_PIN_A, self.ELEVATOR_MOTOR_PIN_B)

        self.pan = ServoDOF(pi, self.WRIST_PAN_PIN)
        self.rotate = MotorPIDDOF(pi, wrist_rotate_motor, wrist_angle_sensor, 1, kp=-.001, ki=-.000, kd=-.0000, MIN=0, MAX=1000)
        self.vertical = MotorPIDDOF(pi, self.elevator_motor, encoder, 0, kp=0.05, ki=0, kd=0, MIN=-9000, MAX=0)

        self.gripper_1 = Gripper(gripper_motor_l)
        self.gripper_2 = Gripper(gripper_motor_r)
        self.__update_q(self.START_Q)
        self.__full_set_position()
        time.sleep(100)
        self.o_curr = FK(self.q)
        return

    def __full_set_position(self):
        self.pan.set_position(0)
        #self.pan.set_position(q[self.PAN_MOTOR])
        self.vertical.set_position(self.q[self.VERTICAL_MOTOR])
        self.rotate.set_position(self.q[self.ROTATE_MOTOR])
        time.sleep(1000)
        self.gripper_1.close()
        self.gripper_2.close()
        return

    def __update_q(self, new_q):
        self.q = new_q
        return

# *** TODO: NEED TO ADD IN CATCHING ERRORS HERE, and dof.py ***

    def parse_text(self, command):
        if command is None or command == '' or command == self.NAME:
            print('Sorry, I did not hear you')
        else:
            command = command.lower()
            command = self.__remove_symbols_and_name(command)
            if 'start' in command or 'stop' in command:
                self.__parse_motion_cmd(command)
            elif 'open' in command or 'close' in command:
                self.__parse_gripper_cmd(command)
            elif 'go to' in command:
                self.__parse_location_cmd(command)
            elif 'power' in command:
                self.__parse_power_cmd(command)
            else:
                self.__parse_relative_cmd(command)
        print('parse return done')
        return

    def __remove_symbols_and_name(self, cmd):
        cmd = re.sub("[^A-Za-z0-9 ]", "", cmd, flags=re.UNICODE)
        replace_map = {self.NAME: '', 'units': '', 'unit': '', '-in': '', 'degrees': '', 'inches': '', 'inch': '',
                       'one': '1', 'two': '2', 'three': '3', 'four': '4', 'five': '5', 'six': '6', 'seven': '7',
                       'eight': '8', 'nine': '9', 'to': '2'
                       }
        for entry in replace_map:
            cmd = cmd.replace(entry, replace_map[entry])
        return cmd

    def __parse_motion_cmd(self, command):
        if 'start' in command:
            if 'moving' in command:
                direction = 1 if self.MOVE_WORD in command else -1
                self.vertical.set_velocity(direction)
                return
            if 'rotating' in command:
                direction = 1 if self.ROTATE_WORD in command else -1
                self.rotate.set_velocity(direction)
                return
            if 'panning' in command:
                direction = 1 if self.PAN_WORD in command else -1
                self.pan.set_velocity(direction)  # NOT IMPLEMENTED
                return
        else:  # stop
            # TODO: figure out stop position, update q, stop for other motors...
            self.vertical.stop()
            self.rotate.stop()
            self.pan.stop()  # NOT IMPLEMENTED
            self.__update_q([self.vertical.get_position(), self.rotate.get_position(), self.pan.get_position()])
        return

    def __parse_gripper_cmd(self, command):  # 'Open gripper x'
        g_num = int(command.split()[-1 + self.OFFSET])
        gripper = self.gripper_1 if g_num == 1 else self.gripper_2
        if 'open' in command and self.gripper_closed[g_num - 1]:  # Array offset
            gripper.open()
            self.gripper_closed[g_num - 1] = False
            return
        if 'close' in command and not self.gripper_closed[g_num - 1]:  # Array offset
            gripper.close()
            self.gripper_closed[g_num - 1] = True
        return

    def __parse_location_cmd(self, command):  # "Go to x, y, z" TODO: see how STT formats to get parse correctly
        # degree to radian conversion handled in FK/IK
        x = int(command.split()[-3 + self.OFFSET])
        y = int(command.split()[-2 + self.OFFSET])
        z = int(command.split()[-1 + self.OFFSET])
        o = [x, y, z]
        self.q = IK(o)
        self.o_curr = o
        return

    def __parse_relative_cmd(self, command):
        print("relative command")
        print(command.split())
        new_relative_pos = int(command.split()[-1 + self.OFFSET])
        if 'move' in command:
            direction = 1 if self.MOVE_WORD in command else -1
            new_relative_pos *= direction
            new_relative_pos *= 1500
            if self.vertical.set_position(self.q[self.VERTICAL_MOTOR] + new_relative_pos):
                print(str(self.q[self.VERTICAL_MOTOR] + new_relative_pos) + " ... ")
                self.q[self.VERTICAL_MOTOR] += new_relative_pos
            print('testing')
            return
        if 'rotate' in command:
            direction = 1 if self.ROTATE_WORD in command else -1
            new_relative_pos *= direction
            new_relative_pos *= 2.78
            if self.rotate.set_position(self.q[self.ROTATE_MOTOR] + new_relative_pos):
                self.q[self.ROTATE_MOTOR] += new_relative_pos
            return
        if 'pan' in command:
            direction = 1 if self.PAN_WORD in command else -1
            new_relative_pos *= direction
            print("panning")
            print(self.q[self.PAN_MOTOR] + new_relative_pos)
            if self.pan.set_position(self.q[self.PAN_MOTOR] + new_relative_pos):
                self.q[self.PAN_MOTOR] += new_relative_pos
            return
        return

    def __parse_power_cmd(self, command):
        self.__update_q(self.START_Q) if 'on' in command else self.__update_q(self.OFF_Q)
        self.__full_set_position()
        return
