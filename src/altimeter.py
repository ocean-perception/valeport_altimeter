#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial


class SonarNotFound(Exception):
    """Sonar port could not be found."""
    pass


class Socket(object):
    """
    Sonar connection
    """
    def __init__(self, port,baudrate):
        """

        :param port:
        :param baudrate:
        """
        print 'connect'
        self.conn = serial.Serial(port=port, baudrate=baudrate)

    def open(self):
        """
        Opens serial connection
        :return:
        """
        self.conn.open()

    def close(self):
        """
        Closes serial connection
        :return:
        """
        self.conn.close()


class VA500(object):
    """
    *VA500* class for sonar altimeter
    """
    def __init__(self, port="/dev/serial", baudrate = 115200):
        """

        :param port:
        :param baudrate: Baud rate, 115200 by default (can be 9600-115200)
        """
        print 'va500init'
        self.port = port
        self.baudrate = baudrate

        self.conn = None
        self.initialized = False

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

    def open(self):
        """
        Initializes sonar connection
        :return:
        """
        print 'open va500'
        if not self.conn:
            try:
                self.conn = Socket(self.port, self.baudrate)
            except OSError as e:
                raise SonarNotFound(self.port,e)

        rospy.loginfo("Initializing sonar altimeter on %s", self.port)
        self.initialized = True

    def close(self):
        print('bybye')
        self.conn.close()


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('valeport_altimeter', log_level=rospy.DEBUG)
    print 'init node'

    port = '/dev/ttyUSB0'
    baudrate = 115200

    with VA500(port,baudrate) as va500_altimeter:
        print 'jij'

    rospy.spin()
