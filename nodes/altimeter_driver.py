#!/usr/bin/env python

import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial
import serial


class VA500:
    # Instrument settings
    GET_SW_VERSION = '#032'
    GET_UNIT_SERIAL_NUM = '#034'
    GET_PCB_SERIAL_NUM = '#136'
    GET_CALIBRATION_DATE = '#138'
    GET_TRANSDUCER_FREQ = '#839'

    # Communication settings
    SET_BAUD_RATE = '#059'
    SET_ADDRESS_485 = '#001'
    ADDRESS_485 = '#002'
    ADDRESS_MODE_CONF = '#005'
    ADDRESS_MODE = '#006'

    # Analogue settings
    SET_VOLTAGE_RANGE = '#094'
    GET_VOLTAGE_RANGE = '#095'
    ANALOG_OUTPUT_TEST = '#096'
    SET_ANALOG_RANGE_LIMIT = '#097'
    GET_ANALOG_RANGE_LIMIT = '#098'

    # Sampling regime
    SINGLE_MEASURE = 'S'
    CONTINUOUS_MEASURE = 'M'
    CONFIGURE = '#'
    SET_MEASURE_MODE = '#039'
    GET_MEASURE_MODE = '#040'
    RUN = '#028'

    # Output format
    GET_OUTPUT_FORMAT = '#089'
    SET_OUTPUT_FORMAT = '#082'

    # Range settings

    SET_RANGE_UNITS = '#021'
    GET_RANGE_UNITS = '#022'
    SET_ERROR_MSG = '#118'
    GET_ERROR_MSG = '#119'
    SET_MAX_RANGE = '#823'
    GET_MAX_RANGE = '#824'
    SET_MIN_RANGE = '#840'
    GET_MIN_RANGE = '#841'
    SET_SOUND_SPEED = '#830'
    GETSOUND_SPEED = '#831'

    # Headers
    DATA_HEADER = "$"
    CONFIGURE_HEADER = ">"

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        timeout=2.0,
        max_range=100.0,
        min_range=1.0
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.0

        # Keep things thread safe
        self.mutex = thread.allocate_lock()
        self.connect()

        res = '$'
        self.send(self.CONFIGURE)
        self.send(self.CONFIGURE, cr=False)
        while self.CONFIGURE_HEADER not in res:
            res = self.recv()
            print 'Waiting for a ">" response:', res
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        self.set_or_get(self.SET_MAX_RANGE, max_range)
        self.set_or_get(self.SET_MIN_RANGE, min_range)
        self.set_or_get(self.SET_MEASURE_MODE, 'M1')
        self.execute(self.RUN)

    def connect(self):
        try:
            print("Connecting to Valeport Altimeter on port", self.port, "...")
            self.port = Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                writeTimeout=self.writeTimeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            print("Connected at", self.baudrate)
            print("VA500 is ready.")

        except SerialException:
            print("Serial Exception:")
            print(sys.exc_info())
            print("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print("Cannot connect to Arduino!")
            os._exit(1)

    def open(self):
        """Open the serial port."""
        self.port.open()

    def close(self):
        """Close the serial port."""
        self.port.close()

    def send(self, cmd, cr=True):
        """This command should not be used on its own: it is called by the execute commands
        below in a thread safe manner.
        """
        #print('Sending command', cmd)
        if cr:
            self.port.write(cmd + '\r\n')
        else:
            self.port.write(cmd)
        self.port.flush()

    def recv(self, timeout=2.0):
        timeout = min(timeout, self.timeout)
        """ This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        """
        c = ""
        value = ""
        attempts = 0
        while c != "\r":
            c = self.port.read(1)
            value += c
            attempts += 1
            if c == self.CONFIGURE_HEADER:
                #print('Received', value)
                return c
            if attempts * self.interCharTimeout > timeout:
                #print('Recv timed out with', value)
                return None

        #print('Received', value)
        value = value.strip("\r").strip('\n')
        return value

    def execute(self, cmd):
        """Thread safe execution of "cmd" returning no value."""
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        try:
            self.send(cmd)
            time.sleep(1)
            reply = self.recv()

            if reply == cmd:
                print('Command', cmd, 'succeeed')
            else:
                print('Command did not succeed', reply)
                return False
        except:
            self.mutex.release()
            print("Exception executing command: " + cmd)
            
        self.mutex.release()
        return True

    def set_or_get(self, cmd, input_value=None):
        """Thread safe execution of "cmd" returning a value."""
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        value = None

        try:
            formated_cmd = cmd
            if input_value is not None:
                if type(input_value) is str:
                    formated_cmd = cmd + ';' + input_value
                else:
                    formated_cmd = cmd + ';' + str(int(input_value))
            self.send(formated_cmd)
            time.sleep(1)
            reply = self.recv()
            value = self.recv()
            ret = self.recv()

            if formated_cmd == reply and ret == self.CONFIGURE_HEADER:
                print('Command', cmd, 'returned', value)
            else:
                print('Command did not succeed')
                print(reply, value, ret)
        except:
            self.mutex.release()
            print("Exception executing command: " + cmd)
            
        self.mutex.release()
        return value


if __name__ == '__main__':
    alt = VA500(
        port='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A107R32L-if00-port0',
        baudrate=115200)

    print("Entering main loop")

    while True:
        res = alt.recv()
	print res
        v = res.split(',')
        print v[1], 'meters'
