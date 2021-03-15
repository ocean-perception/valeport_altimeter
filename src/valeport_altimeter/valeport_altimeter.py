#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import bitstring
import binascii
import select
import datetime
from sensor_msgs.msg import Range

from valeport_altimeter.socket import Socket
from valeport_altimeter.messages import Message
from valeport_altimeter.commands import Command
from valeport_altimeter.replies import Reply
from valeport_altimeter.errors import Error


class VA500(object):
    """
    *VA500* class for sonar altimeter
    """

    MAX_TIMEOUT_COUNT = 5

    def __init__(
            self, 
            port="/dev/ttyUSB0", 
            baudrate = 115200,
            frame_id = 'valeport_altimeter',
            min_range = 0,
            max_range = 10):
        """
        :param port:
        :param baudrate: Baud rate, 115200 by default (can be 9600-115200)
        """

	print("INIT")
        self.port = port
        self.baudrate = baudrate

        self.conn = None
        self.initialized = False

        # Timeout count
        self.timeout_count = 0

        self.max_range = min_range
        self.min_range = max_range

        # Publish extracted data in personalised msg
        self.pub = rospy.Publisher('range', Range, queue_size=1)

        self.range_msg = Range()
        self.range_msg.radiation_type = 2  # Source radiation: sound
        self.range_msg.field_of_view = 0.1 # the size of the arc [rad]
        self.range_msg.header.frame_id = frame_id
        self.range_msg.min_range = self.min_range
        self.range_msg.max_range = self.max_range

    def __enter__(self):
        """
        Initializes for first use
        """
        self.open()
        return self

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
                raise Error.SonarNotFound(self.port,e)

        rospy.loginfo("Initializing sonar altimeter on %s", self.port)
        self.initialized = True


    def close(self):
        self.conn.close()

    def scan(self):
        """
        Sends command to scan
        :return:
        """
        # send here something to verify sonar is connected?
        if not self.initialized:
            raise Error.SonarNotConfigured(self.initialized)
        
        


        # Ask sonar to send a single measurement
        self.conn.send(Message.MEASURE)

        # Get the scan data
        try:
            data = self.get(Message.DATA,wait = 1).payload
            self.range_msg.header.stamp = rospy.Time.now()
            self.range_msg.range = float(data)
            self.pub.publish(self.range_msg)
            self.timeout_count = 0
        except TimeoutError:
            self.timeout_count += 1
            rospy.logdebug("Timeout count: %d", self.timeout_count)
            if self.timeout_count >= self.MAX_TIMEOUT_COUNT:
                # Try to resend paramenters
                self.conn.send(Message.MEASURE)
                self.timeout_count = 0

    def get(self, message = None, wait = 2):
        """
        Sends command and returns reply
        :param message: Message to expect
        :param wait: Seconds to wait until received
        :return:
        """
        # Verify if sonar is initialized
        if not self.initialized:
            raise Error.SonarNotConfigured

        expected_name = Message.to_string(message)
        if message:
            rospy.logdebug("Waiting for %s message", expected_name)
        else:
            rospy.logdebug("Waiting for unlabeled message")

        # Determine end time
        end = datetime.datetime.now() + datetime.timedelta(seconds=wait)

        # Wait until received if a specific message ID is requested, otherwise wait forever
        while message is None or datetime.datetime.now() < end:
            if message is None:
                try:
                    reply = self.conn.get_reply()
                    return reply
                except:
                    break
            else:
                try:
                    reply = self.conn.get_reply(message)
                except:
                    break
                # Verify reply ID if requested
            if reply.id == message:
                #rospy.logdebug("Found %s message", expected_name)
                return reply
            else:
                rospy.logwarn("Received unexpected %s message", reply.name)
        # Timeout
        rospy.logerr("Timed out before receiving message: %s", expected_name)
        raise TimeoutError()


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('valeport_altimeter', log_level=rospy.DEBUG)

    # Add private parameters
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baudrate = rospy.get_param('~baudrate', 115200)
    rate_hz = rospy.get_param('~rate_hz', 1)
    frame_id = rospy.get_param('~frame_id', 'valeport_altimeter')
    min_range_m = rospy.get_param('~min_range_m', '0')
    max_range_m = rospy.get_param('~max_range_m', '10')

    va500_altimeter =  VA500(port, baudrate, frame_id, min_range_m, max_range_m)

    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
	print("SPIN")
        va500_altimeter.scan()
        rospy.spinOnce()
        r.sleep()

