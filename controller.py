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
# from transmitter import transmitter
from E34_2G4D20D import E34_2G4D20D
from select import select

errcode=0

class controller:
    MIN_GAS=5e-2

    # JOYSTICK_Y = 1
    # JOYSTICK_X = 0

    CODE_GAS_Z = 2
    CODE_GAS_RZ = 5
    CODE_GAS = 9
    CODE_BRAKE = 10 

    DELAY_RESTART_MAIN_LOOP=3
    DELAY_RESTART_LOOP_INPUT=3
    DELAY_MAIN_LOOP=0.01
    DELAY_INPUT_LOOP=0.001
    DELAY_START=2
    
    main_loop_alive=True
    loop_input_alive=True

    main_loop_alive_lock = threading.Lock()
    loop_input_alive_lock = threading.Lock()
    controller_state_lock = threading.Lock()

    controller_state = {
        # "joystick_x":0,
        # "joystick_y":0,
        "gas":0,
        "gas_r":0
    }

    is_controller_connected=False

    using_bluetooth=False

    debug=False

    module_E34=None
    controller_name=None
    
    def __init__(self,path_config):
        self.load_config(path_config)
        print(self.config)
        self.using_bluetooth=self.config["using-bluetooth"]

        if self.using_bluetooth:
            self.controller_name=self.config["gamepad-name-bluetooth"]
        else:
            self.controller_name=self.config["gamepad-name-USB"]

        cftx=self.config["transmitter"]        
        self.module_E34=E34_2G4D20D(cftx["device"],cftx["baud-rate"],cftx["pin_m0"],cftx["pin_m1"],cftx["pin_aux"],cftx["parameters"])

    def deinit(self):
        self.module_E34.deinit()
        GPIO.cleanup()

    def init(self):
        GPIO.setmode(GPIO.BOARD)        
        self.module_E34.init()

        self.thread_loop_input = threading.Thread(target=self.loop_input)
        self.thread_loop_input.start()
        
        self.thread_main_loop = threading.Thread(target=self.main_loop)
        self.thread_main_loop.start()

        threading.excepthook = self.thread_excepthook

        self.thread_main_loop.join()

        self.module_E34.deinit()
        GPIO.cleanup()

    def load_config(self,path_config):
        f=open(path_config)
        self.config=json.load(f)
        self.debug=self.config["debug"]
        self.config["transmitter"]["parameters"]=bytearray.fromhex(self.config["transmitter"]["parameters"])

    def thread_excepthook(self,args):
        logging.error(traceback.format_exc())

        with self.main_loop_alive_lock:
            self.main_loop_alive=False

        with self.loop_input_alive_lock:            
            self.loop_input_alive=False

    #transmission speed when writing to serial is slower than it should be, figure out why, for now ok
    def main_loop(self):
        def main_loop_alive():
            with self.main_loop_alive_lock:
                return self.main_loop_alive

        while main_loop_alive():
            try:
                controller_state=None
                is_controller_connected=False

                with self.controller_state_lock:
                    is_controller_connected=self.is_controller_connected

                    if is_controller_connected:
                        controller_state=self.controller_state

                if is_controller_connected:
                    data=(json.dumps(controller_state)+'\n').encode()

                    if self.debug:
                        t_ms_now=time.time_ns() / 1e6

                        self.module_E34.serial_port.write(data)
                        success=self.module_E34.wait_aux_rising_timeout(2000)

                        t_ms_after=time.time_ns() / 1e6
                        print(t_ms_after-t_ms_now)
                    else:
                        self.module_E34.serial_port.write(data)
                        success=self.module_E34.wait_aux_rising_timeout(2000)

                    # self.module_E34.serial_port.write(data)
                    # success=self.module_E34.wait_aux_rising_timeout(2000)
                    

                    if self.debug:
                        print(success)
                        print(data)

                time.sleep(self.DELAY_MAIN_LOOP)
            
            except Exception as ex:
                raise ex

    #Microsoft Xbox Series S|X Controller

    def loop_input(self):
        def loop_input_alive():
            with self.loop_input_alive_lock:
                return self.loop_input_alive

        keys=util.resolve_ecodes_dict(util.find_ecodes_by_regex(r'ABS_(Y|RZ|X|Z|GAS|BRAKE)'))
        list_keys=list(keys)
        key_ev_abs=list_keys[0][0]
        # key_abs_y=next((x for x in list_keys[0][1] if x[0]=="ABS_Y"), None)
        # key_abs_x=next((x for x in list_keys[0][1] if x[0]=="ABS_X"), None)
        
        name_key_abs_rz="ABS_GAS" if self.using_bluetooth else "ABS_RZ"
        name_key_abs_z="ABS_BRAKE" if self.using_bluetooth else "ABS_Z"
        
        key_abs_rz=next((x for x in list_keys[0][1] if x[0]==name_key_abs_rz), None)
        key_abs_z=next((x for x in list_keys[0][1] if x[0]==name_key_abs_z), None)
        
        while loop_input_alive():
            gamepad=None

            try:
                devices = [InputDevice(path) for path in list_devices()]
                device = next((x for x in devices if x.name==self.controller_name), None)

                if device is None:
                    print("device " + self.controller_name + " not found")
                    time.sleep(self.DELAY_RESTART_LOOP_INPUT)
                                        
                    continue
                else:
                    with self.controller_state_lock:
                        self.is_controller_connected=True
                        self.controller_state["gas"]=0
                        self.controller_state["gas_r"]=0

                capabilities=device.capabilities(verbose=True)
                # abs_info_y=next((x for x in capabilities[key_ev_abs] if x[0]==key_abs_y), None)
                # abs_info_x=next((x for x in capabilities[key_ev_abs] if x[0]==key_abs_x), None)
                abs_info_rz=next((x for x in capabilities[key_ev_abs] if x[0]==key_abs_rz), None)                
                abs_info_z=next((x for x in capabilities[key_ev_abs] if x[0]==key_abs_z), None)
                
                if abs_info_rz is None or abs_info_z is None:
                    print("failed fetching one or more device capabilities")

                gamepad = InputDevice(device.path)

                while loop_input_alive():
                    event = gamepad.read_one()

                    if event is not None:
                        # if self.debug:
                        #     print(event)
                        #     print(categorize(event))
                        
                        if event.type == ecodes.EV_ABS:                                                        
                            if (not self.using_bluetooth and event.code == self.CODE_ABS_RZ) or (self.using_bluetooth and event.code == self.CODE_GAS):
                                normalized_rz=min(max(event.value/abs_info_rz[1].max,-1),1)

                                with self.controller_state_lock:
                                    self.controller_state["gas"]=normalized_rz
                            elif (not self.using_bluetooth and event.code == self.CODE_ABS_Z) or (self.using_bluetooth and event.code == self.CODE_BRAKE):
                                normalized_z=min(max(event.value/abs_info_z[1].max,-1),1)

                                with self.controller_state_lock:
                                    self.controller_state["gas_r"]=normalized_z
                            
                    time.sleep(self.DELAY_INPUT_LOOP)
                    # print(self.controller_state)
            except (OSError) as ex:
                print(ex)
            finally:
                with self.controller_state_lock:
                    self.is_controller_connected=False

                if gamepad is not None:
                    gamepad.close()

                if loop_input_alive():
                    time.sleep(self.DELAY_RESTART_LOOP_INPUT)

def print_ex(ex):
    print(''.join(traceback.format_exception(etype=type(ex), value=ex, tb=ex.__traceback__)))

def display_usage():
    print("\nUsage: "+os.path.basename(__file__)+" [XBOX Name]\n")

if __name__ == "__main__":
    argc=len(sys.argv)

    # if argc < 2: 
    #     display_usage()
    #     sys.exit(1)

    # controller_name=sys.argv[1]
    # controller=tank_controller(controller_name)
    # print('here i am nignog')

    path_config="config/config-controller.json"
    ctrlr=controller(path_config)

    try:
        ctrlr.init()
    except Exception as ex:
        logging.error(traceback.format_exc())
    finally:
        ctrlr.deinit()
