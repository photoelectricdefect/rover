#https://python-evdev.readthedocs.io/en/latest/usage.html
#https://raspberry-valley.azurewebsites.net/Map-Bluetooth-Controller-using-Python/

import os
import sys
import time
import json
from multiprocessing import Process
import threading
from evdev import InputDevice, categorize, ecodes, list_devices,util
import json
import traceback
import logging
import RPi.GPIO as GPIO
from E34_2G4D20D import E34_2G4D20D
import numpy as np

# from reciever import reciever

class rover:
    PIN_PWM_MOTOR_LEFT_FRONT = 29
    PIN_PWM_MOTOR_LEFT_BACK = 31

    PIN_PWM_MOTOR_RIGHT_FRONT = 33
    PIN_PWM_MOTOR_RIGHT_BACK = 35

    PIN_MOTOR_LEFT_FRONT_A = 19
    PIN_MOTOR_LEFT_FRONT_B = 21
    PIN_MOTOR_LEFT_BACK_A = 23
    PIN_MOTOR_LEFT_BACK_B = 37

    PIN_MOTOR_RIGHT_FRONT_A = 32
    PIN_MOTOR_RIGHT_FRONT_B = 36
    PIN_MOTOR_RIGHT_BACK_A = 38
    PIN_MOTOR_RIGHT_BACK_B = 40


    # PWM_CHANNEL_MOTOR_LEFT = 0 
    # PWM_CHANNEL_MOTOR_RIGHT = 1 

    PWM_FREQUENCY = 2000 
    # PWM_RESOLUTION = 8

    # MIN_GAS=5e-2

    # JOYSTICK_Y = 1
    # JOYSTICK_RZ = 5

    # BTN_GAS = 9
    # BTN_BRAKE = 10
    # BTN_TR = 311

    # DELAY_RESTART_MAIN_LOOP=3
    # DELAY_RESTART_input_loop=3
    # DELAY_MAIN_LOOP=0.05
    # DELAY_START=2
    
    # main_loop_alive=True
    # rx_loop_alive=True

    # main_loop_alive_lock = threading.Lock()
    # rx_loop_alive_lock = threading.Lock()
    # motion_state_lock = threading.Lock()

    # motion_state = {
    #     "left":0,
    #     "right":0,
    # }

    main_loop_alive=True
    loop_input_alive=True

    main_loop_alive_lock = threading.Lock()
    loop_input_alive_lock = threading.Lock()

    DELAY_RESTART_MAIN_LOOP=3
    DELAY_RESTART_LOOP_INPUT=3
    DELAY_MAIN_LOOP=0.005
    DELAY_INPUT_LOOP=0.001

    module_E34=None
    # controller_name=None
    
    def __init__(self,path_config):
        self.load_config(path_config)
        print(self.config)
        # self.controller_name=self.config["gamepad-name"]
        cftx=self.config["reciever"]        
        self.module_E34=E34_2G4D20D(cftx["device"],cftx["baud-rate"],cftx["pin_m0"],cftx["pin_m1"],cftx["pin_aux"],cftx["parameters"])

    def init(self):
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.PIN_MOTOR_LEFT_FRONT_A, GPIO.OUT)
        GPIO.setup(self.PIN_MOTOR_LEFT_FRONT_B, GPIO.OUT)
        GPIO.setup(self.PIN_MOTOR_LEFT_BACK_A, GPIO.OUT)
        GPIO.setup(self.PIN_MOTOR_LEFT_BACK_B, GPIO.OUT)

        GPIO.setup(self.PIN_MOTOR_RIGHT_FRONT_A, GPIO.OUT)
        GPIO.setup(self.PIN_MOTOR_RIGHT_FRONT_B, GPIO.OUT)
        GPIO.setup(self.PIN_MOTOR_RIGHT_BACK_A, GPIO.OUT)
        GPIO.setup(self.PIN_MOTOR_RIGHT_BACK_B, GPIO.OUT)

        GPIO.setup(self.PIN_PWM_MOTOR_LEFT_FRONT, GPIO.OUT)
        GPIO.setup(self.PIN_PWM_MOTOR_LEFT_BACK, GPIO.OUT)

        GPIO.setup(self.PIN_PWM_MOTOR_RIGHT_FRONT, GPIO.OUT)
        GPIO.setup(self.PIN_PWM_MOTOR_RIGHT_BACK, GPIO.OUT)

        self.pwm_motor_left_front = GPIO.PWM(self.PIN_PWM_MOTOR_LEFT_FRONT, self.PWM_FREQUENCY)
        self.pwm_motor_left_back = GPIO.PWM(self.PIN_PWM_MOTOR_LEFT_BACK, self.PWM_FREQUENCY)
        self.pwm_motor_right_front = GPIO.PWM(self.PIN_PWM_MOTOR_RIGHT_FRONT, self.PWM_FREQUENCY)
        self.pwm_motor_right_back = GPIO.PWM(self.PIN_PWM_MOTOR_RIGHT_BACK, self.PWM_FREQUENCY)

        self.pwm_motor_left_front.start(0)
        self.pwm_motor_left_back.start(0)
        self.pwm_motor_right_front.start(0)
        self.pwm_motor_right_back.start(0)
        
        self.module_E34.init0()

    def deinit(self):
        self.module_E34.deinit()
        GPIO.cleanup()

    def start(self):
        errcode=0

        try:            
            self.thread_loop_input = threading.Thread(target=self.loop_input)
            self.thread_loop_input.start()
            
            self.thread_main_loop = threading.Thread(target=self.main_loop)
            self.thread_main_loop.start()
        
            self.thread_main_loop.join()
            self.thread_loop_input.join()        
        except KeyboardInterrupt as ex:
            errcode=1
        except Exception as ex:
            # print_ex(ex)
            logging.error(traceback.format_exc())
            errcode=1
        finally:
            with self.main_loop_alive_lock:
                self.main_loop_alive=False

            with self.loop_input_alive_lock:            
                self.loop_input_alive=False
            
            sys.exit(errcode)

    def load_config(self,path_config):
        f=open(path_config)
        self.config=json.load(f)
        self.config["reciever"]["parameters"]=bytearray.fromhex(self.config["reciever"]["parameters"])

    def main_loop(self):
        def main_loop_alive():
            with self.main_loop_alive_lock:
                return self.main_loop_alive

        while main_loop_alive():
            try:
                # left=0
                # right=0
                
                # with self.motion_state_lock:
                #     left=self.motion_state["left"]
                #     right=self.motion_state["right"]
                
                # abs_left=abs(left)
                # abs_right=abs(right)

                # self.set_left_motor_stationary()
                # self.set_right_motor_stationary()

                # if abs_left<self.MIN_GAS or abs_right<self.MIN_GAS: 
                #     self.set_left_motor_stationary()
                #     self.set_right_motor_stationary()
                # else:
                #     if(left>0):
                #         self.set_left_motor_counter_clockwise()
                #     else:
                #         self.set_left_motor_clockwise()

                #     if(right>0):
                #         self.set_right_motor_counter_clockwise()
                #     else:
                #         self.set_right_motor_clockwise()

                # self.pwm_left.ChangeDutyCycle(abs_left*100)
                # self.pwm_right.ChangeDutyCycle(abs_right*100)
                time.sleep(self.DELAY_MAIN_LOOP)
            except Exception as ex:
                raise ex
                # print_ex(ex)

                # if main_loop_alive():                
                #     time.sleep(self.DELAY_RESTART_MAIN_LOOP)


    def loop_input(self):
        def loop_input_alive():
            with self.loop_input_alive_lock:
                return self.loop_input_alive
        
        while loop_input_alive():
            try:
                # t_ms_now=time.time_ns() / 1e6

                data=self.module_E34.serial_port.read_until('\n'.encode())
                print(data)

                params = json.loads(data)
                (omega_l,omega_r)=get_control_inputs(params["joystick_x"],params["gas"])

                time.sleep(self.DELAY_INPUT_LOOP)
                # t_ms_after=time.time_ns() / 1e6
                # print(t_ms_after-t_ms_now)
            except (ValueError) as ex:
                continue
            # except (OSError) as ex:
            #     raise ex
                # print(ex)
            # finally:
                # if loop_input_alive():
                #     time.sleep(self.DELAY_RESTART_LOOP_INPUT)


    def get_control_inputs(self,joystick_x,gas):
        sign_joystick_x=np.sign(joystick_x)
        omega_l=0
        omega_r=0

        if sign_joystick_x < 0:
            omega_l=(1+joystick_x)*gas
            omega_r=gas
        elif sign_joystick_x == 0:
            omega_l=gas
            omega_r=gas
        else:
            omega_l=gas
            omega_r=(1-joystick_x)*gas

        omega_l=max(min(omega_l,1),-1)
        omega_r=max(min(omega_r,1),-1)

        return (omega_l,omega_r)


    def set_left_motors_stationary(self): 
        GPIO.output(self.PIN_MOTOR_LEFT_FRONT_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_LEFT_FRONT_B, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_B, GPIO.LOW)

    def set_left_motors_clockwise(self):
        GPIO.output(self.PIN_MOTOR_LEFT_FRONT_A, GPIO.HIGH)
        GPIO.output(self.PIN_MOTOR_LEFT_FRONT_B, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_A, GPIO.HIGH)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_B, GPIO.LOW)

    def set_left_motors_counter_clockwise(self): 
        GPIO.output(self.PIN_MOTOR_LEFT_FRONT_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_LEFT_FRONT_B, GPIO.HIGH)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_B, GPIO.HIGH)



    def set_right_motors_stationary(self): 
        GPIO.output(self.PIN_MOTOR_RIGHT_FRONT_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_RIGHT_FRONT_B, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_RIGHT_BACK_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_RIGHT_BACK_B, GPIO.LOW)

    def set_right_motors_clockwise(self):
        GPIO.output(self.PIN_MOTOR_RIGHT_FRONT_A, GPIO.HIGH)
        GPIO.output(self.PIN_MOTOR_RIGHT_FRONT_B, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_RIGHT_BACK_A, GPIO.HIGH)
        GPIO.output(self.PIN_MOTOR_RIGHT_BACK_B, GPIO.LOW)

    def set_right_motors_counter_clockwise(self): 
        GPIO.output(self.PIN_MOTOR_RIGHT_FRONT_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_RIGHT_FRONT_B, GPIO.HIGH)
        GPIO.output(self.PIN_MOTOR_RIGHT_BACK_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_RIGHT_BACK_B, GPIO.HIGH)


def print_ex(ex):
    print(''.join(traceback.format_exception(etype=type(ex), value=ex, tb=ex.__traceback__)))

# def display_usage():
#     print("\nUsage: "+os.path.basename(__file__)+" [XBOX Name]\n")

if __name__ == "__main__":
    argc=len(sys.argv)

    # if argc < 2: 
    #     display_usage()
    #     sys.exit(1)

    # controller_name=sys.argv[1]
    # controller=tank_controller(controller_name)
    path_config="config/config-rover.json"
    rvr=rover(path_config)
    rvr.init()
    rvr.start()
    rvr.deinit()
