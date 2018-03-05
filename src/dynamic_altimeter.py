#!/usr/bin/env python

import rospy

import serial
import bitstring
import datetime

from Socket import Socket
from Messages import Message
from Commands import Command

import Errors
from sensor_msgs.msg import Range
from dynamic_reconfigure.server import Server
from valeport_altimeter.cfg import ScanConfig


class VA500(object):
    def __init__(self, port="/dev/ttyUSB0", baudrate = 115200):
        self.conn = None
        self.port = port
        self.baudrate = baudrate

        self.port_enabled = None
        self.port_baudrate = None

        self.min_range = 0
        self.max_range = 1000
        srv = Server(ScanConfig, self.config_callback)

    def __enter__(self):
        """
        Initializes for first use
        """
        self.open()

        return self

    def open(self):
        """
        Initializes sonar connection
        :return:
        """
        # Initialize the port
        if not self.conn:
            try:
                self.conn = Socket(self.port, self.baudrate)
            except OSError as e:
                raise Errors.SonarNotFound(self.port,e)
        
        try:
            self.conn.open()
        except:
            pass

        rospy.loginfo("Initializing sonar altimeter on %s", self.port)
        self.initialized = True

        #self.scan()


    def config_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {altimeter_port_enabled}, {altimeter_port_baudrate}""".format(**config))
        self.set_params(**config)
        return config

    def set_params(self,altimeter_port_enabled = None, altimeter_port_baudrate = None, groups = None):
        self.port_enabled = altimeter_port_enabled
        self.port_baudrate = altimeter_port_baudrate
        if self.port_enabled:
            self.open()
        else:
            try:
                self.close()
            except:
                pass

    def scan(self):
        r = rospy.Rate(10)

        if not self.initialized:
            raise Errors.SonarNotConfigured(self.initialized)

        while not rospy.is_shutdown():
            if self.port_enabled:
                #print rospy.Time.now()
                # Ask sonar to send a single measurement
                
                self.conn.send(Message.MEASURE)


                # Get the scan data
                try:
                    data = self.get(Message.DATA,wait = 2)#.payload
                    self.range = float(data)
                except:
                    rospy.logwarn("Error reading stream %s", data)

                # Publish extracted data in personalised msg
                pub2 = rospy.Publisher('range',Range, queue_size=10)
                try:
                    pub2.publish(range = self.range, min_range = self.min_range, max_range = self.max_range)
                except:
                    rospy.logwarn('could not publish range msg')
                    pass

            r.sleep()

    def get(self, message = None, wait = 2):
        """
        Sends command and returns reply
        :param message: Message to expect
        :param wait: Seconds to wait until received
        :return:
        """
        # Verify if sonar is initialized
        if not self.initialized:
            raise Errors.SonarNotConfigured

        expected_name = Message.to_string(message)
        if message:
            rospy.logdebug("Waiting for %s message", expected_name)
        else:
            rospy.logdebug("Waiting for unlabeled message")

        # Determine end time
        end = datetime.datetime.now() + datetime.timedelta(seconds=wait)

        reply = self.conn.get_reply(expected_reply = message,enabled=self.port_enabled)
        return reply

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Cleans up
        :param exc_type:
        :param exc_val:
        :param exc_tb:
        :return:
        """
        self.close()
        rospy.loginfo("Closed sonar altimeter on %s", self.port)

    def close(self):
        if not self.conn:
            try:
                self.conn = Socket(self.port, self.baudrate)
            except OSError as e:
                raise SonarNotFound(self.port,e)
        self.conn.close()




if __name__ == "__main__":
    rospy.init_node('valeport_altimeter', log_level=rospy.DEBUG)

    r = rospy.Rate(10)
    port = '/dev/ttyUSB0'
    baudrate = 115200
    with VA500(port,baudrate) as va500_altimeter:
        va500_altimeter.scan()

    
