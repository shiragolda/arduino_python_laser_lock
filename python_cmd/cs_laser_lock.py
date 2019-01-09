import serial
import struct
import time
import numpy as np
from datetime import datetime
import os
import zmq
import datetime
import sys
import threading


#N_STEPS = 800  # number of voltage steps per scan



ZEROV = 32768

params_default = (1600,

                  2500., 1000.,0.00,
                  ZEROV,
                  1,
                  800)

params_struct_size = 4*10
params_struct_fmt = '<'+'ifffiii'

"""
params =    int ramp amplitude [0]
            float gain_p, gain_i, gain_i2 [1],[2],[3]
            int output_offset [4]
            int scan_state [5]
            int n_steps [6]
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



class zmq_pub_dict:
    """Publishes a python dictionary on a port with a given topic."""

    def __init__(self, port=5552, topic='interferometer'):
        zmq_context = zmq.Context()
        self.topic = topic
        self.pub_socket = zmq_context.socket(zmq.PUB)

        self.pub_socket.bind("tcp://*:%s" % port)
        print('Broadcasting on port {0} with topic {1}'.format(port,
                                                               topic))

    def send(self, data_dict):
        timestamp = time.time()
        send_string = "%s %f %s" % (self.topic, timestamp, repr(data_dict))
        #print(send_string)
        self.pub_socket.send(send_string)

    def close(self):
        self.pub_socket.close()

    def publish_data(self,data):
        try:
            err,corr,msg = data
            data_dict = {'Error (V)': err, 'Correction (V)': corr, 'Message': msg}
            dt = str(time.time())
            self.send(data_dict)
        except:
            print('error retrieving/parsing data')


## Threading
kbd_input = ''
new_input = True

def commandListener():
    global kbd_input, new_input
    kbd_input = raw_input()
    new_input = True


publish = False

def publishOn():
    cslock.ser.write(b'm')
    global publish
    publish = True
    print("Publishing...")

def publishOff():
    cslock.ser.write(b'm')
    global publish
    publish = False
    print("Publishing off")


publisher = zmq_pub_dict(5553,'cs_laser')

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

    def set_scan_state(self,scan_state):
        self.params[5] = int(scan_state)
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
        self.params[0] = int(new_amp - new_amp % self.params[6])
        self.set_params()

    def close(self):
        self.ser.close()


def print_params():
        """ Print the current parameters on the microcontroller. """
        print('Ramp amplitude = {0:.3f} V,'.format(ToVoltage(cslock.params[0])))
        print("Output pffset = {0:.3f} V".format(ToVoltage(cslock.params[4]-ZEROV)))
        print('Servo Gain Parameters: P = {0:.0f}, I = {1:.0f}, I2 = {2:.0f}'.format(cslock.params[1],cslock.params[2],cslock.params[3]))
        print('Scan On/Off: {0:.0f}'.format(cslock.params[5]))

def scan_amp(new_amplitude):
    """Set scan amplitude on the microcontroller in Volts.
    Scan amplitude will be made a multiple of N_STEPS."""
    cslock.update_amplitude(new_amplitude)

def scan_steps(new_steps):
    cslock.params[6] = int(new_steps)
    old_amplitude = cslock.params[0]
    cslock.update_amplitude(ToVoltage(old_amplitude))

def lock():
    global p_state,i_state,i2_state
    cslock.set_scan_state(0)
    print("Locked")

def unlock():
    cslock.set_scan_state(1)
    print("Unlocked - Scanning")

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

# def publish_data():
#     publisher = zmq_pub_dict(5553,'cs_laser')
#     cslock.ser.write(b'm')
#     local_accumulator = 0
#     try:
#         while True:
#             err,corr = cslock.get_data()
#             data = (err,corr)
#             publisher.publish_data(data)

#     except KeyboardInterrupt:
#         pass
#     cslock.ser.write(b'm')
#     publisher.close()


def quit():
    print("Loop exited")
    publishOff()
    loop_running = False

def start():
    print("Loop starting")
    publishOn()
    loop_running = True

if __name__ == '__main__':
    cslock = CsLock()


loop_running = True
local_accumulator = 0
while(loop_running):
    try:
        if(publish):
            try:
                err,corr = cslock.get_data()
                local_accumulator+=err
                msg = 'ok'
                if(np.abs(local_accumulator)>10000.0):
                    msg = 'out of lock'
                data = (err,corr,msg)
                publisher.publish_data(data)
            except:
                pass

        if(new_input):
            try:
                exec(kbd_input)
            except:
                print("Invalid input")
            new_input = False
            listener = threading.Thread(target=commandListener)
            listener.start()

    except(KeyboardInterrupt):
        print("Loop exited")
        cslock.ser.write(b'm')
        loop_running = False

    except:
        pass

publisher.close()
cslock.close()
