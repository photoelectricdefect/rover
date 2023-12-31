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
import collections

class rover:
    PIN_PWM_MOTOR_LEFT_FRONT = 33
    PIN_PWM_MOTOR_LEFT_BACK = 31

    PIN_PWM_MOTOR_RIGHT_FRONT = 35
    PIN_PWM_MOTOR_RIGHT_BACK = 29

    PIN_MOTOR_LEFT_FRONT_A = 32
    PIN_MOTOR_LEFT_FRONT_B = 36

    PIN_MOTOR_LEFT_BACK_A = 23
    PIN_MOTOR_LEFT_BACK_B = 37

    PIN_MOTOR_RIGHT_FRONT_A = 38
    PIN_MOTOR_RIGHT_FRONT_B = 40

    PIN_MOTOR_RIGHT_BACK_A = 19
    PIN_MOTOR_RIGHT_BACK_B = 21

    PWM_FREQUENCY = 2000 

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

    DELAY_RESTART_MAIN_LOOP=3
    DELAY_RESTART_LOOP_INPUT=3
    DELAY_MAIN_LOOP=0.01
    DELAY_INPUT_LOOP=0.001

    DELIM_MSG='\n'.encode()[0]
    SIZE_BUFFER_MSG=128

    debug=False

    module_E34=None
    
    def __init__(self,path_config):
        self.load_config(path_config)
        print(self.config)
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

        threading.excepthook = self.thread_excepthook

        self.thread_loop_input = threading.Thread(target=self.loop_input)
        self.thread_loop_input.start()
        
        self.thread_main_loop = threading.Thread(target=self.main_loop)
        self.thread_main_loop.start()
    
        self.thread_main_loop.join()

    def load_config(self,path_config):
        f=open(path_config)
        self.config=json.load(f)
        self.debug=self.config["debug"]
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
                (omega_L,omega_R)=self.get_controller_inputs(controller_state)
                self.motion_state["omega_L"]=omega_L
                self.motion_state["omega_R"]=omega_R

            if self.motion_state["omega_R"] > 0.001 and self.debug:
                print(self.motion_state)

            omega_L=self.motion_state["omega_L"]
            omega_R=self.motion_state["omega_R"]

            if omega_L < self.OMEGA_MIN and omega_R < self.OMEGA_MIN:
                self.set_left_motors_stationary()
                self.set_right_motors_stationary()
            else:
                self.set_right_motors_clockwise()
                self.set_left_motors_counter_clockwise()
            
            self.set_pwm_left_motors(omega_L*100)
            self.set_pwm_right_motors(omega_R*100)                
            
            time.sleep(self.DELAY_MAIN_LOOP)

    def loop_input(self):
        def loop_input_alive():
            with self.loop_input_alive_lock:
                return self.loop_input_alive
        
        buffer_msg = collections.deque(maxlen=self.SIZE_BUFFER_MSG)

        while loop_input_alive():
            try:
                data=None

                if self.debug:
                    t_ms_now=time.time_ns() / 1e6

                    data=self.module_E34.serial_port.read(self.module_E34.serial_port.in_waiting)

                    t_ms_after=time.time_ns() / 1e6
                    print(t_ms_after-t_ms_now)
                else:
                    # data=self.module_E34.serial_port.read_until('\n'.encode()) #very slow for some reason, do not do
                    data=self.module_E34.serial_port.read(self.module_E34.serial_port.in_waiting)

                n_msg=0

                for i in range(len(data)):
                    buffer_msg.append(data[i])

                    if data[i] == self.DELIM_MSG:
                        n_msg+=1

                if n_msg > 0:
                    buffer_msg_out = collections.deque(maxlen=self.SIZE_BUFFER_MSG)
                    n_delim_detected=0
                    n_bytes_relocated=0

                    while len(buffer_msg) > n_bytes_relocated:
                        val=buffer_msg.pop()

                        if val == self.DELIM_MSG:
                            n_delim_detected+=1

                        if n_delim_detected == 0:
                            buffer_msg.appendleft(val)
                            n_bytes_relocated+=1
                        elif n_delim_detected == 1:
                            buffer_msg_out.append(val)

                    if len(buffer_msg_out) > 0:
                        msg=bytearray(len(buffer_msg_out))
                        pos_msg=0

                        while len(buffer_msg_out) > 0:
                            msg[pos_msg]=buffer_msg_out.pop()
                            pos_msg+=1

                        if self.debug:
                            print(msg)

                        params = json.loads(msg)

                        with self.controller_state_lock:
                            self.is_input_timed_out=False
                            self.controller_state["gas"]=params["gas"]
                            self.controller_state["gas_r"]=params["gas_r"]

                time.sleep(self.DELAY_INPUT_LOOP)
            except (ValueError) as ex:
                print("err parsing json message from reciever")
                
                with self.controller_state_lock:
                    self.is_input_timed_out=True
            except (KeyError) as ex:
                pass

    def get_controller_inputs(self,controller_state):
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
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_B, GPIO.HIGH)

    def set_left_motors_counter_clockwise(self): 
        GPIO.output(self.PIN_MOTOR_LEFT_FRONT_A, GPIO.LOW)
        GPIO.output(self.PIN_MOTOR_LEFT_FRONT_B, GPIO.HIGH)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_A, GPIO.HIGH)
        GPIO.output(self.PIN_MOTOR_LEFT_BACK_B, GPIO.LOW)

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
        self.pwm_motor_left_front.ChangeDutyCycle(duty*0.5)
        self.pwm_motor_left_back.ChangeDutyCycle(duty)

    def set_pwm_right_motors(self,duty): 
        self.pwm_motor_right_front.ChangeDutyCycle(duty*0.5)
        self.pwm_motor_right_back.ChangeDutyCycle(duty*0.5)

def print_ex(ex):
    print(''.join(traceback.format_exception(etype=type(ex), value=ex, tb=ex.__traceback__)))

def display_usage():
    pass

if __name__ == "__main__":
    argc=len(sys.argv)

    path_config="config/config-rover.json"
    rvr=rover(path_config)
    
    try:
        rvr.init()
    except Exception as ex:
        logging.error(traceback.format_exc())
    finally:
        rvr.deinit()
