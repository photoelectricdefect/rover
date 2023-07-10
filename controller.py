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

class controller:
    MIN_GAS=5e-2

    JOYSTICK_Y = 1
    JOYSTICK_X = 0
    GAS_Z = 2
    GAS_RZ = 5

    DELAY_RESTART_MAIN_LOOP=3
    DELAY_RESTART_LOOP_INPUT=3
    DELAY_MAIN_LOOP=0.001
    DELAY_INPUT_LOOP=0.001
    DELAY_START=2
    
    main_loop_alive=True
    loop_input_alive=True

    main_loop_alive_lock = threading.Lock()
    loop_input_alive_lock = threading.Lock()
    motion_state_lock = threading.Lock()

    motion_state = {
        "joystick_x":0,
        "joystick_y":0,
        "gas":0,
        "gas_r":0
    }

    module_E34=None
    controller_name=None
    
    def __init__(self,path_config):
        self.load_config(path_config)
        print(self.config)
        self.controller_name=self.config["gamepad-name"]
        cftx=self.config["transmitter"]        
        self.module_E34=E34_2G4D20D(cftx["device"],cftx["baud-rate"],cftx["pin_m0"],cftx["pin_m1"],cftx["pin_aux"],cftx["parameters"])

    #TODO: exception handling,timing between control and input thread is fucked because too many events are read (read events less often),...
    def start(self):
        errcode=0

        try:
            self.module_E34.init()

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
        self.config["transmitter"]["parameters"]=bytearray.fromhex(self.config["transmitter"]["parameters"])

    def main_loop(self):
        def main_loop_alive():
            with self.main_loop_alive_lock:
                return self.main_loop_alive

        while main_loop_alive():
            try:
                motion_state=None

                with self.motion_state_lock:
                    motion_state=self.motion_state

                data=(json.dumps(motion_state)+'\n').encode()
                self.module_E34.serial_port.write(data)
                # success=self.module_E34.wait_aux_rising_timeout(2000)

                # self.module_E34.write(data,2)
                print(data)
                # print(err)

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

        keys=util.resolve_ecodes_dict(util.find_ecodes_by_regex(r'ABS_(Y|RZ|X|Z)'))
        list_keys=list(keys)
        key_ev_abs=list_keys[0][0]
        key_abs_y=next((x for x in list_keys[0][1] if x[0]=="ABS_Y"), None)
        key_abs_x=next((x for x in list_keys[0][1] if x[0]=="ABS_X"), None)
        key_abs_rz=next((x for x in list_keys[0][1] if x[0]=="ABS_RZ"), None)
        key_abs_z=next((x for x in list_keys[0][1] if x[0]=="ABS_Z"), None)
        
        while loop_input_alive():
            gamepad=None

            try:
                devices = [InputDevice(path) for path in list_devices()]
                device = next((x for x in devices if x.name==self.controller_name), None)

                if device is None:
                    time.sleep(self.DELAY_RESTART_LOOP_INPUT)
                    continue

                capabilities=device.capabilities(verbose=True)
                abs_info_y=next((x for x in capabilities[key_ev_abs] if x[0]==key_abs_y), None)
                abs_info_x=next((x for x in capabilities[key_ev_abs] if x[0]==key_abs_x), None)
                abs_info_rz=next((x for x in capabilities[key_ev_abs] if x[0]==key_abs_rz), None)
                abs_info_z=next((x for x in capabilities[key_ev_abs] if x[0]==key_abs_z), None)
                gamepad = InputDevice(device.path)

                # print(abs_info_y)
                # print(abs_info_x)
                # print(abs_info_rz)
                # print(abs_info_z)

                while loop_input_alive():
                    event = gamepad.read_one()
                    # r, w, x = select([gamepad], [], [])
                    # events = gamepad.read()
                    
                    # r, w, x = select([gamepad], [], [])
                    # event = None 
                    
                    # .read() will surely return a list of events now.
                    # for ev in gamepad.read():
                    #     event = ev                    
                    
                    # # continue
                    # print("asas")
                    # event = next(events, None)
                    # print(event)
                    # print("asasasssssssssssssssss")

                    if event is not None:
                        # print(categorize(event))
                        # print(event)

                        if event.type == ecodes.EV_ABS:
                            if event.code == self.JOYSTICK_Y and abs_info_y is not None:
                                normalized_y=min(max(event.value/abs_info_y[1].max,-1),1)
                                
                                with self.motion_state_lock:
                                    self.motion_state["joystick_y"]=normalized_y
                            elif event.code == self.JOYSTICK_X and abs_info_x is not None:
                                normalized_x=min(max(event.value/abs_info_x[1].max,-1),1)
                                
                                with self.motion_state_lock:
                                    self.motion_state["joystick_x"]=normalized_x
                            elif event.code == self.GAS_RZ and abs_info_rz is not None:
                                normalized_rz=min(max(event.value/abs_info_rz[1].max,-1),1)

                                with self.motion_state_lock:
                                    self.motion_state["gas"]=normalized_rz
                            elif event.code == self.GAS_Z and abs_info_z is not None:
                                normalized_z=min(max(event.value/abs_info_z[1].max,-1),1)

                                with self.motion_state_lock:
                                    self.motion_state["gas_r"]=normalized_z
        

                    time.sleep(self.DELAY_INPUT_LOOP)
                    # print(self.motion_state)
            except (OSError) as ex:
                print(ex)
            finally:
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
    ctrlr.start()
