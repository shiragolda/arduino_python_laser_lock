import serial
import struct
import time
import numpy as np
from datetime import datetime
import os
import zmq_logging


#N_STEPS = 800  # number of voltage steps per scan



ZEROV = 32768

params_default = (1600,

                  0., 0.,0.00,
                  ZEROV,
                  0,
                  0,
                  0,
                  1,
                  800)

params_struct_size = 4*10
params_struct_fmt = '<'+'ifffiiiiii'

"""
params =    int ramp amplitude [0]
            float gain_p, gain_i, gain_i2 [1],[2],[3]
            int output_offset [4]
            int p_gain_on [5]
            int integrator_on [6]
            int integrator2_on [7]
            int scan_state [8]
            int n_steps [9]
"""

p_state = 0
i_state = 0
i2_state = 0


# float ToVoltage(float bits) {
#   return (bits-32768)/6553.6;
# }

# float ToBits(float voltage) {
#   return voltage*6553.6+32768;
# }

def ToVoltage(bits):
    return float(bits/6553.6)

def ToBits(voltage):
    return int((voltage*6553.6))

class CsLock:
    """Control the uc32 set up for laser locking.

    Requires:
    A uc32 microcontroller flashed with the .ino code
    """


    def __init__(self, serialport='COM14'):
        self.serialport = serialport
        self.ser = serial.Serial(serialport, baudrate=115200, timeout=4.0)

        time.sleep(4)  # wait for microcontroller to reboot
        self.params = list(params_default)
        self.set_params()
        time.sleep(0.1)


    def idn(self):
        """Read id of the microcontroller and return read string."""
        self.ser.write(b'i')
        return self.ser.readline()

    def load_from_eeprom(self):
        self.ser.write(b'r')

    def save_to_eeprom(self):
        self.ser.write(b'w')

    def get_params(self):
        """Get params structure from the microcontroller and store it locally."""
        write_string = b'g'
        self.ser.write(write_string)
        data = self.ser.read(params_struct_size)
        data_tuple = struct.unpack(params_struct_fmt, data)
        self.params = list(data_tuple)
        return data_tuple

    def set_params(self):
        """Set params on the microcontroller."""
        data = struct.pack(params_struct_fmt, *self.params)
        self.ser.write(b's'+data)


    def set_lock_state(self,pst,ist,i2st):
        """ Set the on/off state of proportional gain, integral gain, integral^2 gain """
        self.params[5] = int(pst)
        self.params[6] = int(ist)
        self.params[7] = int(i2st)
        self.set_params()

    def set_scan_state(self,scan_state):
        self.params[8] = int(scan_state)
        self.set_params()

    def get_data(self):
        s = self.ser.readline()
        try:
            err,corr = s.split(',')
            err = float(err)
            corr = float(corr)
        except:
            err = 0
            corr = 0
        return err,corr
    def update_amplitude(self,new_amp):
        new_amp = ToBits(new_amp)
        self.params[0] = int(new_amp - new_amp % self.params[9])
        self.set_params()


def print_params():
        """ Print the current parameters on the microcontroller. """
        print('Ramp amplitude = {0:.3f} V,'.format(ToVoltage(cslock.params[0])))
        print("Output pffset = {0:.3f} V".format(ToVoltage(cslock.params[4]-ZEROV)))
        print('Servo Gain Parameters: P = {0:.0f}, I = {1:.0f}, I2 = {2:.0f}'.format(cslock.params[1],cslock.params[2],cslock.params[3]))
        print('Servo Gain On/Off:  P = {0:.0f}, I = {1:.0f}, I2 = {2:.0f}'.format(cslock.params[5],cslock.params[6],cslock.params[7]))
        print('Scan On/Off: {0:.0f}'.format(cslock.params[8]))

def scan_amp(new_amplitude):
    """Set scan amplitude on the microcontroller in Volts.
    Scan amplitude will be made a multiple of N_STEPS."""
    cslock.update_amplitude(new_amplitude)

def scan_steps(new_steps):
    cslock.params[9] = int(new_steps)
    old_amplitude = cslock.params[0]
    cslock.update_amplitude(ToVoltage(old_amplitude))

def lock():
    global p_state,i_state,i2_state
    cslock.set_scan_state(0)
    cslock.set_lock_state(p_state,i_state,i2_state)
    print("Locked")

def unlock():
    cslock.set_scan_state(1)
    cslock.set_lock_state(0,0,0)
    print("Unlocked - Scanning")

def toggle_p():
    global p_state,i_state,i2_state
    p_state = int(not p_state)
    if(p_state): print('Proportional gain on')
    else: print("Proportional gain off")
    cslock.set_lock_state(p_state,i_state,i2_state)

def toggle_i():
    global p_state,i_state,i2_state
    i_state = int(not i_state)
    if(i_state): print('Integral gain on')
    else: print("Integral gain off")
    cslock.set_lock_state(p_state,i_state,i2_state)

def toggle_i2():
    global p_state,i_state,i2_state
    i2_state = int(not i2_state)
    if(i2_state): print('Integral^2 gain on')
    else: print("Integral^2 gain off")
    cslock.set_lock_state(p_state,i_state,i2_state)

def gain_p( p_gain):
    """ Set the proportional gain. Use a positive gain to lock to a positive slope error signal.  """
    cslock.params[1] = p_gain
    cslock.set_params()
    print("Proportional gain updated to:" + str(p_gain))

def gain_i(i_gain):
    """ Set the integral gain """
    cslock.params[2] = i_gain
    cslock.set_params()
    print("Integral gain updated to:" + str(i_gain))

def gain_i2(i2_gain):
    """ Set the integral^2 gain """
    cslock.params[3] = i2_gain
    cslock.set_params()
    print("Integral^2 gain updated to:" + str(i2_gain))

def output_offset(offset):
    """ Set the output offset in V """
    cslock.params[4] = int(ToBits(offset)+ZEROV)
    cslock.set_params()

def increase_offset():
    cslock.params[4] += int(ToBits(0.002))
    cslock.set_params()

def decrease_offset():
    cslock.params[4] -= int(ToBits(0.002))
    cslock.set_params()

def publish_data():
    publisher = zmq_logging.zmq_pub_dict(5553,'cs_laser')
    cslock.ser.write(b'm')
    try:
        while True:
            err,corr = cslock.get_data()
            data = (err,corr)
            publisher.publish_data(data)
    except KeyboardInterrupt:
        pass
    cslock.ser.write(b'm')
    publisher.close()

def close():
    #self.set_lock_state(0)
    cslock.ser.close()

if __name__ == '__main__':
    cslock = CsLock()

