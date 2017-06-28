#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import bitstring


class SonarNotFound(Exception):
    """Sonar port could not be found."""
    pass

class Command(object):
    """
    Sonar commands
    """
    def __init__(self, id, payload = None):
        """
        Construct command object
        :param id:
        :param payload:
        """
        self.id = id
        self.payload = payload if payload else bitstring.BitStream()

    def serialize(self):
        """
        Construct string of bytes to send to sonar
        :return:
        """
        values = {
            "id": self.id,
            "payload_length": self.payload.length,
            "payload": self.payload
        }

        serial_format = (
            "0x23, uint:8=id, 0x3B, bits:payload_length=payload, 0x0D0A"
        )

        message = bitstring.pack(serial_format, **values)

        return message.tobytes()

class Socket(object):
    """
    Sonar connection
    """
    def __init__(self, port,baudrate):
        """

        :param port:
        :param baudrate:
        """
        self.conn = serial.Serial(port=port, baudrate=baudrate)

    def open(self):
        """
        Opens serial connection
        :return:
        """
        print 'send'
        self.conn.open()

    def close(self):
        """
        Closes serial connection
        :return:
        """
        self.conn.close()

    def send(self, message, payload = None):
        """

        :param message:
        :param payload:
        :return:
        """
        cmd = Command(message, payload)
        self.conn.write(cmd.serialize())


class VA500(object):
    """
    *VA500* class for sonar altimeter
    """
    def __init__(self, port="/dev/serial", baudrate = 115200):
        """

        :param port:
        :param baudrate: Baud rate, 115200 by default (can be 9600-115200)
        """
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
        rospy.loginfo("Closed sonar altimeter on %s", self.port)

    def open(self):
        """
        Initializes sonar connection
        :return:
        """
        if not self.conn:
            try:
                self.conn = Socket(self.port, self.baudrate)
            except OSError as e:
                raise SonarNotFound(self.port,e)

        rospy.loginfo("Initializing sonar altimeter on %s", self.port)
        self.initialized = True
        self.conn.send(032)


    def close(self):
        self.conn.close()


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('valeport_altimeter', log_level=rospy.DEBUG)

    port = '/dev/ttyUSB0'
    baudrate = 115200

    with VA500(port,baudrate) as va500_altimeter:
        print 'jij'

    rospy.spin()
