"""Logs the error signal of the rb laser servo."""

import time
import zmq
import datetime
import sys
import numpy as np
import serial



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
        print(send_string)
        self.pub_socket.send(send_string)

    def close(self):
        self.pub_socket.close()

    def publish_data(self,data):
        try:
            err,corr = data
            data_dict = {'Error (V)': err, 'Correction (V)': corr}
            dt = str(time.time())
            self.send(data_dict)
        except:
            print('error')







