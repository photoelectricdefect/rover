import serial
import time
import RPi.GPIO as GPIO
import threading

class E34_2G4D20D():
    PINS_BY_MODE = {
        "half-duplex": {
            "pin_m0": GPIO.LOW,
            "pin_m1": GPIO.LOW
        },
        "full-duplex": {
            "pin_m0": GPIO.HIGH,
            "pin_m1": GPIO.LOW
        },
        "reservation": {
            "pin_m0": GPIO.LOW,
            "pin_m1": GPIO.HIGH
        },
        "setting": {
            "pin_m0": GPIO.HIGH,
            "pin_m1": GPIO.HIGH
        }
    }

    MODES=["half-duplex","full-duplex","reservation","setting"]

    DT_S_SERIAL_OPEN=2
    TIMEOUT_S_SERIAL_WRITE=2
    TIMEOUT_S_SERIAL_READ=2
    DT_MS_INIT_E34=2000
    DT_S_MODE_SET_E34=0.002

    serial_port=None

    mode=None

    is_aux_high=False
    cv_aux = threading.Condition()

    device=None
    baud_rate=0
    pin_m0=0
    pin_m1=0
    pin_aux=0
    parameters=None

    def __init__(self, device, baud_rate, pin_m0, pin_m1, pin_aux, parameters):
        self.device=device
        self.baud_rate=baud_rate
        self.pin_m0=pin_m0
        self.pin_m1=pin_m1
        self.pin_aux=pin_aux
        self.parameters=parameters

    def init(self):
        self.mode=None
        self.is_aux_high = True

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_aux, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pin_m0, GPIO.OUT)
        GPIO.setup(self.pin_m1, GPIO.OUT)

        self.serial_port = serial.Serial(port=self.device, baudrate=self.baud_rate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.serial_port.timeout=self.TIMEOUT_S_SERIAL_READ
        self.serial_port.timeout_write=self.TIMEOUT_S_SERIAL_READ

        time.sleep(self.DT_S_SERIAL_OPEN)
                
        self.wait_aux_high_timeout(self.DT_MS_INIT_E34)

        GPIO.add_event_detect(self.pin_aux,GPIO.RISING)
        # GPIO.add_event_callback(self.pin_aux,self.on_aux_rise)

        self.set_mode("setting",2)
        self.write(bytearray([0xC0])+self.parameters,2)
        self.write(bytearray([0xC1,0xC1,0xC1]),2)        
        params=self.read(6,2)
        
        # print(params)

        if self.parameters != params[1:] or params is None:
            raise Exception("error fetching params from module") 

        self.set_mode("half-duplex",2)

        print("E34_2G4D20D module initialized")

    #temporary, remove
    def init0(self):
        self.mode=None
        self.is_aux_high = True

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_aux, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pin_m0, GPIO.OUT)
        GPIO.setup(self.pin_m1, GPIO.OUT)

        self.serial_port = serial.Serial(port=self.device, baudrate=self.baud_rate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.serial_port.timeout=self.TIMEOUT_S_SERIAL_READ
        self.serial_port.timeout_write=self.TIMEOUT_S_SERIAL_READ

        time.sleep(self.DT_S_SERIAL_OPEN)
                
        self.wait_aux_high_timeout(self.DT_MS_INIT_E34)

        GPIO.add_event_detect(self.pin_aux,GPIO.RISING)
        # GPIO.add_event_callback(self.pin_aux,self.on_aux_rise)

        # self.set_mode("setting",2)
        # self.write(bytearray([0xC0])+self.parameters,2)
        # self.write(bytearray([0xC1,0xC1,0xC1]),2)        
        # params=self.read(6,2)
        
        # if self.parameters != params[1:] or params is None:
        #     raise Exception("error fetching params from module") 

        # self.set_mode("half-duplex",2)

    def wait_aux_high(self):
        while GPIO.input(self.pin_aux) != GPIO.HIGH:
            continue

    def wait_aux_high_timeout(self,timeout_ms):
        t_ms_now=time.time_ns() / 1e6

        while GPIO.input(self.pin_aux) != GPIO.HIGH:
            dt_ms=(time.time_ns() / 1e6) - t_ms_now

            if dt_ms > timeout_ms:
                return False

            continue

        return True

    def wait_aux_rising_timeout(self,timeout_ms):
        t_ms_now=time.time_ns() / 1e6

        while not GPIO.event_detected(self.pin_aux):
            dt_ms=(time.time_ns() / 1e6) - t_ms_now

            if dt_ms > timeout_ms:
                return False

            continue

        return True

    # def is_ready(self):
    #     return self.is_aux_high

    def write(self,data,timeout_s):
        # with self.cv_aux:
            # is_ready=self.cv_aux.wait_for(self.is_ready,timeout_s)
            
            # if is_ready:
        is_ready=self.wait_aux_rising_timeout(2)

        if is_ready:
            # self.is_aux_high = False
            self.serial_port.write(data)

        return is_ready

    def read(self,length,timeout_s):
        # with self.cv_aux:
        #     is_ready=self.cv_aux.wait_for(self.is_ready,timeout_s)

        #     if is_ready:
                # self.is_aux_high = False

        return self.serial_port.read(length)

    def read_until(self,delim,timeout_s):
        # with self.cv_aux:
        #     is_ready=self.cv_aux.wait_for(self.is_ready,timeout_s)

        #     if is_ready:
                # self.is_aux_high = False
                
        return self.serial_port.read_until(delim)

    # def on_aux_rise(self,event):
        # print("rise")

        # with self.cv_aux:
        #     self.is_aux_high = True
        #     self.cv_aux.notify_all()


    def deinit(self):
        self.serial_port.close()
        GPIO.remove_event_detect(self.pin_aux)
        GPIO.cleanup()

    def set_mode(self,mode,timeout_s):
        # with self.cv_aux:
            # is_ready=self.cv_aux.wait_for(self.is_ready,timeout_s)
            
            # if is_ready:
        # is_ready=self.wait_aux_rising_timeout(2)

        if self.wait_aux_rising_timeout(2):
            pins_state = self.PINS_BY_MODE[mode]
            GPIO.output(self.pin_m0, pins_state["pin_m0"])
            GPIO.output(self.pin_m1, pins_state["pin_m1"])
            
            if self.mode == "setting" and mode != "setting":
                # self.is_aux_high = False
                # GPIO.event_detected(self.pin_aux)
                # GPIO.wait_for_edge(self.pin_aux, GPIO.RISING)
                if not self.wait_aux_rising_timeout(2):
                    return False

            time.sleep(self.DT_S_MODE_SET_E34)
            self.mode = mode

            return True
        
        return False