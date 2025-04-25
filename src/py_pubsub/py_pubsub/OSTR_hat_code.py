from smbus2 import SMBus
import time

# Author: Ben Keppers
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

# Library for ADC, PWM Expander, and Motor drivers on the raspberry pi hat

# Many of the below constants are redefined in the sections of
# code that use them . . . eventually this code should be fixed
# so that isn't necessary


## Temp sensor
# Run "sudo dtoverlay w1-gpio gpiopin=4" to enable 1-wire. Required to use temp sensor


## IMU

ADDRIMU = 0x68
freq_divider = 0x04
g = 9.8 # gravity acceleration (m/s^2)


## ADC

# ADC I2C Address
ADDRADC = 0x48

# ADC pins
MD0I = 0    # Motor 0 current
MD1I = 1    # Motor 1 current
SADC2 = 2   # Servo feedback
SADC3 = 3   # servo feedback
VBATT = 7   # Battery voltage, scaled to 0-3.3V

# Command Byte
SEINPUTS = 0x80     # Differential inputs
DEINPUTS = 0x00     # Single-ended inputs
CSSHIFT = 4         # shift to apply to desired pin
PDMODE = 0x04       # power-down mode (Internal Reference OFF and A/D Converter ON)

# Single-Ended PIN Look Up Table -- to convert ADC pin to ADC channel
SEPINLUT = [0x8, 0xC, 0x9, 0xD, 0xA, 0xE, 0xB, 0xF]

# ADC Conversion Constants
ADCREF = 3.3

# Motor current monitoring
AIPROPI = 0.000450      # motor driver current monitor current gain
# ADC count to motor current (A)
ADCM0OI = (ADCREF / 255) / (1800 * AIPROPI)     #Motor 0, 1.8k RPROPI for 4.07 A limit
ADCM1OI = (ADCREF / 255) / (6800 * AIPROPI)     #Motor 1, 6.8k RPROPI for 1.08 A limit


# Battery voltage monitoring
RBATL = 10000
RBATU = 56000
# ADC count to battery voltage (V)
ADCBATTC = (RBATL + RBATU)/RBATL * (ADCREF/255)


## PWM

# PWM chip I2C Address
ADDRPWM = 0x1C

# Define various registers
MODE1 = 0x00
MODE2 = 0x01
PWM0 = 0x02
LEDOUT0 = 0x0C  # pins 0-3
LEDOUT1 = 0x0D  # pins 4-7

# Define control bytes
CTRL = 0x00
CTRLAI = 0x80   # Autoincrement flag


## Motor Drivers

# Driver 0 PWM pins
M0P1 = 1
M0P2 = 0

# Driver 1 PWM pins
M1P1 = 3
M1P2 = 2

# Motor driver modes
MDMRC = 0       # Reverse, Coast -- doesn't seem to make motor move
MDMFC = 1       # Forward, Coast -- doesn't seem to make motor move
MDMFB = 2       # Forward, Brake
MDMRB = 3       # Reverse, Brake

#defaut delay
DEFDELAY = 0.1


## ADC

class ADC:
    def __init__(self, addr, i2cbus):
        self.addr = addr        # I2C address of chip
        self.i2cbus = i2cbus
    
    def read(self, pin):
        #print("ADC read")
        #print(f"Control byte: {(SEPINLUT[pin] << CSSHIFT) | PDMODE}")
        self.i2cbus.write_byte(self.addr, (SEPINLUT[pin] << CSSHIFT) | PDMODE)  # Initiate a read on pin
        time.sleep(DEFDELAY)
        return self.i2cbus.read_byte(self.addr)                                  # Get ADC reading and return it


## PWM Generator

class PWMExpander:
    def __init__(self, addr, i2cbus):
        self.addr = addr        # I2C address of chip
        self.i2cbus = i2cbus
        #control byte: 1000 0000 0x80
        # MODE1 register: 1000 0000 0x80
        # MODE2 register: 0000 0101 0x05
        #print(f"MODE: {self.i2cbus.read_i2c_block_data(self.addr, CTRLAI, 18)}")
        #print(f"MODE1: {self.i2cbus.read_i2c_block_data(self.addr, 0, 1)}")
        #print(f"MODE2: {self.i2cbus.read_i2c_block_data(self.addr, 1, 1)}")
        self.i2cbus.write_i2c_block_data(self.addr, CTRLAI, [0x00, 0x1D])
        time.sleep(DEFDELAY)
        #print(f"MODE: {self.i2cbus.read_i2c_block_data(self.addr, CTRLAI, 18)}")
        
    # set PWM duty cycle and set pin mode to PWM
    # pin: 0x00-0x07
    # duty: 0-256
    def setPWM(self, pin, duty):
        #print("PWM setPWM")
        #self.setPINMODE(pin, 0)                                     # turn pin off
        #print(f"Control byte: {PWM0 + pin}, Duty: {duty}")
        duty = 256-duty
        if (duty > 256 or duty < 0):
            print("duty out of range")
            return 1
        self.i2cbus.write_byte_data(self.addr, PWM0 + pin, duty)    # write duty to pin PWM register
        time.sleep(DEFDELAY)
        #print(f"MODE: {self.i2cbus.read_i2c_block_data(self.addr, CTRLAI, 18)}")
        self.setPINMODE(pin, 2)                                     # enable PWM on pin
        
    
    # set pin output mode
    # 0 - on; 1 - off; 2 - PWM
    def setPINMODE(self, pin, mode):
        #print("PWM setPINMODE")
        pinStates = self.i2cbus.read_i2c_block_data(self.addr, CTRLAI | (LEDOUT0),2)      # get current state
        #print(f"Control Byte: {CTRLAI | (LEDOUT0 + int(pin/4))}")
        #print(f"Current pinStates: {pinStates}")
        pinStates[int(pin/4)] = pinStates[int(pin/4)] & (0xFF^(0x3 << 2*(pin%4)))   # clear bits associated with given pin
        pinStates[int(pin/4)] = pinStates[int(pin/4)] | (mode << 2*(pin%4))         # set bits associated with given pin to mode
            
        #print(f"New pinStates: {pinStates}")
        self.i2cbus.write_i2c_block_data(self.addr, CTRLAI | (LEDOUT0), pinStates)      # set new state
        time.sleep(DEFDELAY)
        #print(f"MODE: {self.i2cbus.read_i2c_block_data(self.addr, CTRLAI, 18)}")
        

## Motor Drivers

class motorDriver:
    # pwmexpander - PWMExpander object that controls the IO expander
    # pin1 - PWMExpander pin connected to EN_IN1 pin
    # pin2 - PWMExpander pin connected to PH_IN2 pin
    def __init__(self, pwmexpander, pin1, pin2):
        self.pin = [0,0]
        self.pin[0] = pin1
        self.pin[1] = pin2
        self.pwmexpander = pwmexpander
        
    # mode - left = constant pin state; right = constant pin
    # 0 - 00 - coast; reverse
    # 1 - 01 - coast; forward
    # 2 - 10 - brake; forward
    # 3 - 11 - brake; reverse
    # duty = 0-256, 0 = stop, 256 = full speed
    def go(self, mode, duty):
        #print("Motor go")
        #print(f"Pin: {self.pin[mode%2]}, Mode: {int(mode/2)}")
        self.pwmexpander.setPINMODE(self.pin[mode%2], int(mode/2))     # set value of constant pin
        
        if duty == 0:
            #print(f"Pin: {self.pin[1-mode%2]}, Mode: 1")
            self.pwmexpander.setPINMODE(self.pin[1-mode%2], 1)
            
        else:
            #print(f"Pin: {self.pin[1-mode%2]}, Duty: {duty}")
            self.pwmexpander.setPWM(self.pin[1-mode%2], duty)
            
    
    # mode - 0 = coast; 1 = brake
    def stop(self, mode):
        #print("Motor stop")
        self.pwmexpander.setPINMODE(self.pin[0], mode)
        self.pwmexpander.setPINMODE(self.pin[1], mode)
