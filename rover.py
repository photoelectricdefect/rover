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

    PIN_MOTOR_LEFT_FRONT_A = 32
    PIN_MOTOR_LEFT_FRONT_B = 36
    PIN_MOTOR_LEFT_BACK_A = 37
    PIN_MOTOR_LEFT_BACK_B = 23

    PIN_MOTOR_RIGHT_FRONT_A = 38
    PIN_MOTOR_RIGHT_FRONT_B = 40
    PIN_MOTOR_RIGHT_BACK_A = 19
    PIN_MOTOR_RIGHT_BACK_B = 21


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

    controller_state_lock = threading.Lock()
    controller_state = {
        "gas":0,
        "gas_r":0
    }

    motion_state_lock = threading.Lock()
    motion_state = {
        "omega_L":0,
        "omega_R":0
    }

    OMEGA_MIN=1e-3

    main_loop_alive=True
    loop_input_alive=True

    main_loop_alive_lock = threading.Lock()
    loop_input_alive_lock = threading.Lock()

    TIMEOUT_S_SERIAL_READ=0.3

    is_input_timed_out=False
    timestamp_ns_input_timed_out=-1
    motion_state_saved=None

    T_ROVER_MOTION_HALT_S=0.8
    T_ROVER_MOTION_HALT_NS=T_ROVER_MOTION_HALT_S*1e9

    # N_STEPS_ROVER_MOTION_HALT_MAX=50

    n_steps_rover_motion_halt=0

    DELAY_RESTART_MAIN_LOOP=3
    DELAY_RESTART_LOOP_INPUT=3
    DELAY_MAIN_LOOP=0.01
    DELAY_INPUT_LOOP=0.001

    module_E34=None
    # controller_name=None
    
    def __init__(self,path_config):
        self.load_config(path_config)
        print(self.config)
        # self.controller_name=self.config["gamepad-name"]
        cftx=self.config["reciever"]        
        self.module_E34=E34_2G4D20D(cftx["device"],cftx["baud-rate"],cftx["pin_m0"],cftx["pin_m1"],cftx["pin_aux"],cftx["parameters"])

    def deinit(self):
        self.module_E34.deinit()
        GPIO.cleanup()

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
        
        self.module_E34.init()
        self.module_E34.serial_port.timeout=self.TIMEOUT_S_SERIAL_READ

        self.thread_loop_input = threading.Thread(target=self.loop_input)
        self.thread_loop_input.start()
        
        self.thread_main_loop = threading.Thread(target=self.main_loop)
        self.thread_main_loop.start()
    
        threading.excepthook = self.thread_excepthook

        self.thread_main_loop.join()

    def load_config(self,path_config):
        f=open(path_config)
        self.config=json.load(f)
        self.config["reciever"]["parameters"]=bytearray.fromhex(self.config["reciever"]["parameters"])

    def thread_excepthook(self,args):
        logging.error(traceback.format_exc())

        with self.main_loop_alive_lock:
            self.main_loop_alive=False

        with self.loop_input_alive_lock:            
            self.loop_input_alive=False

    def main_loop(self):
        def main_loop_alive():
            with self.main_loop_alive_lock:
                return self.main_loop_alive

        while main_loop_alive():
            try:
                is_input_timed_out=False
                controller_state=None

                with self.controller_state_lock:
                    is_input_timed_out=self.is_input_timed_out
                    
                    if not is_input_timed_out:
                        controller_state=self.controller_state

                if is_input_timed_out:
                    timestamp_ns_now=time.time_ns()

                    if self.timestamp_ns_input_timed_out < 0:
                        self.timestamp_ns_input_timed_out = timestamp_ns_now
                        self.motion_state_saved = self.motion_state
                    elif self.timestamp_ns_input_timed_out < timestamp_ns_now:
                        dt_input_timed_out_ns=timestamp_ns_now-self.timestamp_ns_input_timed_out

                        if dt_input_timed_out_ns < self.T_ROVER_MOTION_HALT_NS:
                            weight=dt_input_timed_out_ns/self.T_ROVER_MOTION_HALT_NS
                            self.motion_state["omega_L"]=self.motion_state_saved["omega_L"]*(1-weight)
                            self.motion_state["omega_R"]=self.motion_state_saved["omega_R"]*(1-weight)
                        else:
                            self.motion_state["omega_L"]=0
                            self.motion_state["omega_R"]=0
                else:
                    self.timestamp_ns_input_timed_out = -1
                    (omega_L,omega_R)=self.get_control_inputs(controller_state)
                    self.motion_state["omega_L"]=omega_L
                    self.motion_state["omega_R"]=omega_R

                # self.set_left_motors_clockwise()
                # self.set_right_motors_counter_clockwise()
                # self.set_pwm_left_motors(100)
                # self.set_pwm_right_motors(100)

                # continue

                if self.motion_state["omega_R"] > 0.001:
                    print(self.motion_state)

                omega_L=self.motion_state["omega_L"]
                omega_R=self.motion_state["omega_R"]

                if omega_L < self.OMEGA_MIN and omega_R < self.OMEGA_MIN:
                    print("AAAAAAAAAAAA")
                    self.set_left_motors_stationary()
                    self.set_right_motors_stationary()
                else:
                    print("BBBBBBBBBBBB")
                    self.set_left_motors_clockwise()
                    self.set_right_motors_counter_clockwise()
                
                self.set_pwm_left_motors(omega_L*100)
                self.set_pwm_right_motors(omega_R*100)

                    # DT_ROVER_MOTION_HALT_S=0.8
                    # N_STEPS_ROVER_MOTION_HALT=50

                    # omega_L=motion_state["omega_L"]
                    # omega_R=motion_state["omega_R"]
                                        
                    # motion_state = {
                    #     "omega_L":0,
                    #     "omega_R":0,
                    # }

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

                # self.controller_state["gas"]=1
                # self.controller_state["gas_r"]=1


                data=self.module_E34.serial_port.read_until('\n'.encode())
                # print(data)

                params = json.loads(data)

                with self.controller_state_lock:
                    self.is_input_timed_out=False
                    self.controller_state["gas"]=params["gas"]
                    self.controller_state["gas_r"]=params["gas_r"]

                time.sleep(self.DELAY_INPUT_LOOP)
                # t_ms_after=time.time_ns() / 1e6
                # print(t_ms_after-t_ms_now)
            except (ValueError) as ex:
                with self.controller_state_lock:
                    self.is_input_timed_out=True
            except (KeyError) as ex:
                pass

    # Update
    def get_control_inputs(self,controller_state):
        # sign_joystick_x=np.sign(joystick_x)
        # omega_l=0
        # omega_r=0

        # if sign_joystick_x < 0:
        #     omega_l=(1+joystick_x)*gas
        #     omega_r=gas
        # elif sign_joystick_x == 0:
        #     omega_l=gas
        #     omega_r=gas
        # else:
        #     omega_l=gas
        #     omega_r=(1-joystick_x)*gas

        gas=controller_state["gas"]
        gas_r=controller_state["gas_r"]

        omega_l=max(min(gas,1),-1)
        omega_r=max(min(gas_r,1),-1)

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



    def set_pwm_left_motors(self,duty):
        self.pwm_motor_left_front.ChangeDutyCycle(duty)
        self.pwm_motor_left_back.ChangeDutyCycle(duty)
        print("duty_L")
        print(duty)

    def set_pwm_right_motors(self,duty): 
        self.pwm_motor_right_front.ChangeDutyCycle(duty)
        self.pwm_motor_right_back.ChangeDutyCycle(duty)
        print("duty_R")
        print(duty)



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
    
    try:
        rvr.init()
    except Exception as ex:
        logging.error(traceback.format_exc())
    finally:
        print("deinited")
        rvr.deinit()
