#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import bitstring
import binascii
import select
import datetime
from valeport_altimeter.msg import Valeport_Altimeter


class SonarNotFound(Exception):
    """Sonar port could not be found."""
    pass

class DataNotSent(Exception):
    """Sonar is not sending information."""
    pass

class PacketIncomplete(Exception):
    """Packet is incomplete."""
    pass

class PacketCorrupted(Exception):
    """Packet is corrupt."""
    pass

class SonarNotConfigured(Exception):
    """Sonar is not configured for scanning."""
    pass

class TimeoutError(Exception):
    """Communication timed out."""
    pass

class Message(object):
    """
    Enumeration of available messages

    """

    CONFIGURATION_PARAM = 0

    # Instrument settings
    SW_VERSION       = 0x23303332 #032
    UNIT_SERIAL_NUM  = 0x23303334 #034
    PCB_SERIAL_NUM   = 0x23313336 #136
    CALIBRATION_DATE = 0x23313338 #138
    TRANSDUCER_FREQ  = 0x23383339 #839

    # Communication settings
    BAUD_RATE         = 0x23303539 #059
    SET_ADDRESS_485   = 0x23303031 #001
    ADDRESS_485       = 0x23303032 #002
    ADDRESS_MODE_CONF = 0x23303035 #005
    ADDRESS_MODE      = 0x23303036 #006

    # Analogue settings
    SET_VOLTAGE_RANGE       = 0x23303934 #094
    VOLTAGE_RANGE           = 0x23303935 #095
    ANALOG_OUTPUT_TEST      = 0x23303936 #096
    SET_ANALOG_RANGE_LIMIT  = 0x23303937 #097
    ANALOG_RANGE_LIMIT      = 0x23303938 #098

    # Sampling regime
    SINGLE_MEASURE      = 0x53 # hex value of 'S'
    DATA                = '$' #
    MEASURE             = 0x4D # hex value of 'M'
    CONFIGURE           = 0x23 # hex value of #
    READY_2_CONFIGURE   = '>'
    SET_MEASURE_MODE    = 0x23303339 #039
    OPERATING_MODE      = 0x23303430 #040
    RUN                 = 0x23303238 #028

    # Output format
    OUTPUT_FORMAT       = 0x23303238 #089
    SET_OUTPUT_FORMAT   = 0x23303832 #082

    # Range settings
    SET_RANGE_UNITS     = 0x23303231 #021
    RANGE_UNITS         = 0x23303232 #022
    SET_ERROR_MSG       = 0x23313138 #118
    ERROR_MSG           = 0x23313139 #119
    MAX_RANGE           = 0x23383233 #823
    MINIMUM_RANGE       = 0x23383431 #841
    CHANGE_SOUND_SPEED  = 0x23383330 #830
    SOUND_SPEED         = 0x23383331 #831

    @classmethod
    def to_string(cls,id):
        """
        Get human-readable name corresponding to message id
        :param cls:
        :param id: message ID
        :return:
        """
        for attr, value in cls.__dict__.iteritems():
            if value == id:
                return attr
        else:
            return None

    @classmethod
    def from_string(cls, name):
        """
        Gets message ID corresponding to human-readable name.
        :param name: Human-readable string
        :return:
        """
        if hasattr(cls,name):
            return cls.__getattribute__(name)
        else:
            return None

class Reply(object):
    """
    Parses and verifies reply packages
    """
    def __init__(self, bitstream, id = 0):
        """

        :param bitstream:
        """
        self.bitstream = bitstream
        self.id = id
        self.name = "<unknown>"
        self.payload = None
        self.dataformat = 'NMEA'
        self.dataunits = None
        self.parse()

    def parse(self):
        """
        Parses packet into header, message ID and payload
        :return:
        """
        try:
            if self.bitstream:
                # Parse message header
                self.bitstream.bytepos = 0

                if self.bitstream.endswith("\n"):
                    header = self.bitstream.read("uint:8")

                else:
                    raise PacketIncomplete("Packet does not end with carriage return")

                if self.bitstream.find('0x 50 52 56 41 54',bytealigned=True): # If 'PRVAT' text in bitstream
                    self.dataformat = 'NMEA'
                else:
                    self.dataformat = 'TRITECH'

                if self.dataformat=='NMEA':
                    # go to first comma
                    self.bitstream.bytepos = self.bitstream.find('0x2C', bytealigned = True)[0]/8 + 1
                    self.payload = self.bitstream.read('bytes:6')
                    #skip comma
                    self.bitstream.read('bytes:1')
                    self.dataunits = self.bitstream.read('bytes:1')


                if self.dataformat=='TRITECH':
                    self.bitstream.bytepos = 0
                    self.payload = self.bitstream.read('bytes:6')
                    self.dataunits = self.bitstream.read('bytes:1')

                rospy.logdebug("%s message has payload %s",Message.to_string(self.id),self.payload)

            else:
                pass

        except ValueError as e:
            raise PacketCorrupted("Unexpected error", e)




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

        # The len must be multiple of 4 bits to convert unambiguously

        id_len = self.id.bit_length()

        while (id_len % 4)!= 0:
            id_len += 1

        values = {
            "id": self.id,
            "id_len": id_len,
            "payload": self.payload,
            "payload_len": len(self.payload)
        }

        serial_format = (
            "uint:id_len=id, bits:payload_len=payload, 0x0D0A"
        )

        message = bitstring.pack(serial_format, **values)

        rospy.logdebug("Sent command '0x%s'", message.hex)

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
        rospy.logdebug("Sending %s: %s", Message.to_string(message), payload)
        self.conn.write(cmd.serialize())

    def get_reply(self, expected_reply = None):
        """
        Waits for and returns reply
        :return:
        """
        try:
            # Wait for the header character
            # Don't put anything in this while, because if losses packets if you do so
            if expected_reply:
                print 'expected_reply'
                while not self.conn.read() == expected_reply:
                    pass
            else:
                print 'no expected reply'
                while self.conn.read() == None:
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
                message_id = Message.CONFIGURATION_PARAM

            # Convert each caracter from received string stream in the bitstream
            while True:
                current_line = self.conn.readline()
                for char in current_line:
                    # This saves what is inside ord(char) in a two digit hex
                    packet.append("0x{:02X}".format(ord(char)))
                print packet

                # Try to parse
                try:
                    reply = Reply(packet, id = message_id)
                    break
                except PacketIncomplete:
                    rospy.logdebug("Received packet incomplete")
                    # Keep looking
                    continue
                    rospy.logdebug("Received %s: %s", reply.name, reply.payload)
            return reply

        except select.error as (code,msg):
             if code == errno.EINTR:
                 raise KeyboardInterrupt()
             raise




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
        self.configured = False

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
                raise SonarNotFound(self.port,e)

        rospy.loginfo("Initializing sonar altimeter on %s", self.port)
        self.initialized = True

        # The sonar sends information with previous configuration when it is plugged in,
        # so it throws exception when no data is received

        try:
            self.get(Message.DATA)

        except OSError as e:
            rospy.loginfo("Sonar is not sending data on init. Try reseting it")
            raise DataNotSent(self.conn, e)

        rospy.loginfo("Preparing sonar to be configured...")
        self.configure()
        self.configured = True
        rospy.loginfo("Sonar ready to be configured")
        self.conn.send(Message.SW_VERSION)
        self.get()
        self.scan()

        #self.conn.send(Message.TRANSDUCER_FREQ)
        #self.conn.get_reply()


    def close(self):
        self.conn.close()

    def configure(self):
        self.conn.send(Message.CONFIGURE)
        rospy.logdebug('%s sent', Message.to_string(Message.CONFIGURE))
        self.get(Message.READY_2_CONFIGURE)

    def scan(self):
        """
        Sends command to scan
        :return:
        """
        # send here something to verify sonar is connected?
        if not self.initialized:
            raise SonarNotConfigured(self.initialized)
        # Timeout count
        timeout_count = 0
        MAX_TIMEOUT_COUNT = 5

        # Scan until stopped
        self.preempted = False
        while not self.preempted:
            # Preempt on ROS shutdown
            if rospy.is_shutdown():
                self.preempt()
                return
            # Ask sonar to send a single measurement
            self.conn.send(Message.MEASURE)

            # Get the scan data
            try:
                data = self.get(None,wait = 1).payload
                timeout_count = 0
            except TimeoutError:
                timeout_count += 1
                rospy.logdebug("Timeout count: %d", timeout_count)
                if timeout_count >= MAX_TIMEOUT_COUNT:
                    # Try to resend paramenters
                    self.conn.send(Message.MEASURE)
                    timeout_count = 0
                # Try again
                continue
        # Publish extracted data in personalised msg
        print 'data:'
        print data



    def get(self, message = None, wait = 2):
        """
        Sends command and returns reply
        :param message: Message to expect
        :param wait: Seconds to wait until received
        :return:
        """
        # Verify if sonar is initialized
        if not self.initialized:
            raise SonarNotConfigured

        expected_name = Message.to_string(message)
        if message:
            rospy.logdebug("Waiting for %s message", expected_name)
        else:
            rospy.logdebug("Waiting for unlabeled message")

        # Determine end time
        end = datetime.datetime.now() + datetime.timedelta(seconds=wait)

        # Wait until received if a specific message ID is requested, otherwise wait forever
        while message is None or datetime.datetime.now() < end:
            try:
                if message is None:
                    reply = self.conn.get_reply()
                    return reply
                else:
                    reply = self.conn.get_reply(message)
                # Verify reply ID if requested
                if reply.id == message:
                    #rospy.logdebug("Found %s message", expected_name)
                    return reply
                else:
                    rospy.logwarn("Received unexpected %s message", reply.name)
            except PacketCorrupted, serial.SerialException:
                # Keep trying
                continue
        # Timeout
        rospy.logerr("Timed out before receiving message: %s", expected_name)
        raise TimeoutError()

    def preempt(self):
        """
        Preempts a scan in progress
        :return:
        """
        rospy.logwarn("Preempting scan...")
        self.preempted = True


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('valeport_altimeter', log_level=rospy.DEBUG)

    port = '/dev/ttyUSB0'
    baudrate = 115200

    with VA500(port,baudrate) as va500_altimeter:
        pass

