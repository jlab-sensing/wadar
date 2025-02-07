"""
This program is free software: you can redistribute it and/or modify it under the terms of the 
GNU
General Public License as published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without
even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.
The GNU General Public License can be found at <
http://www.gnu.org/licenses/
>.
"""
"""
Linux interface to Mini
-
Circuits matrix switch
RC
-
2SP6T
-
A12.
Written by: Tim Dunker
E
-
mail: tdu [at] justervesenet [dot] no
OpenPGP key: 0x4FDBC497
This programme is based on the script 'Minimalist Linux interface to Mini Circuits USB power 
meter', copyright (C) 2017, Rigetti & Co. Inc., distributed 
with a GPL licence.

Basic usage: 
from MiniCircuits_Switch import MiniCircuits_Switch
sw = MiniCircuits_Switch()
# Switch to channel 6 on switch A:  
sw.switch(1,6)
# Switch to channel 3 on switch B:
sw.switch(2,3)
# De-energize switch A:
sw.switch(1,0) """

import string
import sys
import threading
import time
import numpy as np
import libusb1
import usb1

################################################################################
#
# Class to operate a Mini-Circuits RC-2SP6T-A12 switch.
#
################################################################################

class MiniCircuits_Switch():
    """
    Operate a Mini Circuits matrix switch RC-2SP6T-A12.
    """
    # Vendor ID of Mini Circuits:
    _VENDOR_ID = 0x20CE
    # Product ID of the RF switch box:
    _PRODUCT_ID = 0x22
    # Length of a message in bytes:
    _MSG_LEN = 64

    _GET_DEVICE_MODEL_NAME = 40
    _GET_DEVICE_SERIAL_NUMBER = 41
    _GET_FIRMWARE = 99
    _SET_SP6T_SWITCH = 12

    def __init__(self):
        """
        Connect to the USB RF matrix switch.
        """
        # Configure threading
        self.lock = threading.Lock()
        # Configure the USB target:
        self.context = usb1.USBContext()
        self.handle = self.context.openByVendorIDAndProductID(0x20CE,0x22)
        if self.handle is None:
            raise RuntimeError('No USB RF matrix switch found.')
        if self.handle.kernelDriverActive(0):
            self.handle.detachKernelDriver(0)
        self.handle.resetDevice()
        self.handle.claimInterface(0)
        self.model = self._query_string([MiniCircuits_Switch._GET_DEVICE_MODEL_NAME,])
        self.sernum = self._query_string([MiniCircuits_Switch._GET_DEVICE_SERIAL_NUMBER,])

    def _query(self, cmd):
        """
        Issue the provided command, get and validate the response, and return the rest of the
        response.
        """
        self.cmd = bytearray([0]*MiniCircuits_Switch._MSG_LEN)
        self.cmd[:len(cmd)] = cmd
        self.lock.acquire()
        threading.Thread(target=self._write).start()
        while self.lock.locked():
            time.sleep(1e-6)
        if self.nsent != len(self.cmd):
            sys.exit(1)
        self.lock.acquire()
        threading.Thread(target=self._read).start()
        while self.lock.locked():
            time.sleep(10e-6)
        if len(self.response) != MiniCircuits_Switch._MSG_LEN:
            sys.exit(1)
        return self.response

    def _query_string(self, cmd, response_length=None):
        """
        Issue the provided command and return the string response from the device.
        If the response length is not provided, use a terminating null character to extract
        the
        string.
        """
        resp = self._query(cmd)
        if response_length is None:
            try:
                response_length = resp.index(bytearray([0]))
            except:
                raise RuntimeError('No terminating null character in response: %s' % resp)
        resp = resp[1:response_length-1]
        return str(resp)

    def _read(self):
        """
        Interrupt task required to read from the USB device.
        """
        self.response = ''
        try:
            self.response = self.handle.interruptRead(endpoint=1, length=MiniCircuits_Switch._MSG_LEN, timeout=1000)
        finally:
            self.lock.release()

    def _write(self):
        """
        Interrupt task required to write to the USB device."""
        self.nsent = -1
        try:
            self.nsent = self.handle.interruptWrite(endpoint=1, data=self.cmd, timeout=50)
        finally:
            self.lock.release()

    def get_device_model_name(self):
        """
        Read the device model name of the RF matrix switch.
        """
        resp = self._query_string([MiniCircuits_Switch._GET_DEVICE_MODEL_NAME,])
        try:
            return str(resp)
        except:
            raise RuntimeError('Device model name readout not valid: "%s"' % resp)

    def get_device_serial_number(self):
        """
        Read the serial number of the RF matrix switch.
        """
        resp = self._query_string([MiniCircuits_Switch._GET_DEVICE_SERIAL_NUMBER,])
        try:
            return str(resp)
        except:
            raise RuntimeError('Serial number readout not valid: "%s"' % resp)

    def get_firmware(self):
        """
        Read the firmware of the RF matrix switch.
        """
        resp = self._query([MiniCircuits_Switch._GET_FIRMWARE,])[5:7]
        try:
            return resp
        except:
            raise RuntimeError('Firmware readout not valid: "%s"' % resp)

    def switch(self, switch, state):
        """
        Set the switch "sw" to channel "state".
        """
        resp = self._query_string([MiniCircuits_Switch._SET_SP6T_SWITCH, switch, state,])
        try:
            return str(resp)
        except:
            raise RuntimeError('Switch state readout not valid: "%s"' % resp)




