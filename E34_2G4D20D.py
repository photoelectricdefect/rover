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

    BAUD_RATE_MODE_SETTING = 9600

    DT_S_SERIAL_OPEN=2
    TIMEOUT_S_SERIAL_WRITE=2
    TIMEOUT_S_SERIAL_READ=2
    DT_MS_INIT_E34=2000
    DT_S_MODE_SET_E34=0.002

    N_TRIES_CHECK_PARAMS=10

    serial_port=None

    mode=None

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
        GPIO.setup(self.pin_aux, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pin_m0, GPIO.OUT)
        GPIO.setup(self.pin_m1, GPIO.OUT)

        self.serial_port = serial.Serial(port=self.device, baudrate=self.BAUD_RATE_MODE_SETTING, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.serial_port.timeout=self.TIMEOUT_S_SERIAL_READ
        self.serial_port.write_timeout =self.TIMEOUT_S_SERIAL_WRITE

        time.sleep(self.DT_S_SERIAL_OPEN)

        err=False        

        while True:
            if not self.wait_aux_high_timeout(self.DT_MS_INIT_E34):
                err=True
                break

            time.sleep(self.DT_S_MODE_SET_E34)
            self.set_mode("setting")

            GPIO.add_event_detect(self.pin_aux,GPIO.RISING)

            self.serial_port.write(bytearray([0xC0])+self.parameters)

            if not self.wait_aux_rising_timeout(2000):
                err=True
                break
    
            is_params_ok=False

            for i in range(self.N_TRIES_CHECK_PARAMS):
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()

                self.serial_port.write(bytearray([0xC1,0xC1,0xC1]))

                if not self.wait_aux_rising_timeout(2000):
                    is_params_ok=True
                    break

                params=self.serial_port.read(6)

                if not (self.parameters != params[1:] or params is None):
                    is_params_ok=True
                    break            

            if not is_params_ok:
                err=True
                break
                
            self.set_mode("half-duplex")        
            
            if not self.wait_aux_rising_timeout(2000):
                err=True
                break

            time.sleep(self.DT_S_MODE_SET_E34)
            self.serial_port.baudrate=self.baud_rate
            
            break

        if err:
            raise Exception("error initializing module") 

        print("E34_2G4D20D module initialized")

    def deinit(self):
        if self.serial_port is not None:
            self.serial_port.close()
        
        GPIO.remove_event_detect(self.pin_aux)

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

    def set_mode(self,mode):
        self.mode = mode
        pins_state = self.PINS_BY_MODE[self.mode]
        GPIO.output(self.pin_m0, pins_state["pin_m0"])
        GPIO.output(self.pin_m1, pins_state["pin_m1"])
