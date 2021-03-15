#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import select
import bitstring
from valeport_altimeter.replies import Reply
from valeport_altimeter.commands import Command
from valeport_altimeter.messages import Message

class Socket(object):
    """
    Sonar connection
    """
    def __init__(self, port,baudrate):
        """

        :param port:
        :param baudrate:
        """
        self.conn = serial.Serial(timeout=2)
        self.conn.port = port
        self.conn.baudrate = baudrate

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

    def send(self, message, payload = None, command = None):
        """
        :param message:
        :param payload:
        :return:
        """
        cmd = Command(message, payload,command)
        rospy.logdebug("Sending %s: %s", Message.to_string(message), command)
        self.conn.write(cmd.serialize())

    def _readline(self):
        eol = b'*'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.conn.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)

    def get_reply(self, expected_reply = None, enabled = False):
        """
        Waits for and returns reply
        :return:
        """
        try:
            # Wait for the header character
            # Don't put anything in this while, because if losses packets if you do so
            if expected_reply:
                while not self.conn.read() == '$':
                    pass

            # Initialize empty packet where the received stream will be saved
            packet = bitstring.BitStream()

            if expected_reply == '>':
                message_id = Message.READY_2_CONFIGURE
                rospy.logdebug("Sonar altimeter in configuration mode")
                reply = Reply(packet.append("0x{:02X}".format(ord('>'))), id=message_id)
                return reply

            elif expected_reply == '$':
                message_id = Message.DATA
                rospy.logdebug("Received valid packet with sensor data")
            else:
                rospy.logdebug("Received packet with configuration parameters")
                message_id = Message.CONFIGURATION_PARAM

            # Convert each caracter from received string stream in the bitstream
            current_line = []
            while enabled:
                try:
                    # Read the message sent by altimeter. It always has length 22
                    current_line.append(self.conn.read(22))     
                    # Separate fields by commas
                    char = str(current_line).split(',')
                    measurement = char[1]
                    break
                except:
                    continue

        except select.error as (code, msg):
             if code == errno.EINTR:
                 raise KeyboardInterrupt()
             raise

        rospy.logdebug("Received: %s", measurement)
        return measurement #reply
