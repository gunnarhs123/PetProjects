# -*- coding: utf-8 -*-

##########################################
# deap_sea_8620.py
# Use of pymodbus library
# The DeapSea 8620 Control Module is a HW - module which is connected to an electric generator
# Definition of class DeapSea_8620 for control through DeapSea 8620-HW-module with the Modbus-protocol
# Used to read state, status of the electric genearator and turn the electric generator ON/OFF
# Returns state (OFF/WAITING/ON) and status tuple of the equipment/device (specific values, alarms and errors)
# Sets state (ON/OFF) for the device
##########################################

import logging
import json
import time
# We have to deal with 16 bit and 23 unsigned integers let us import numpy to make it easier
import numpy as np
from datetime import datetime, date, timedelta

from its.its_constants import *

#import pymodbus
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.transaction import ModbusSocketFramer

# Symbolic constants for detail info (Universal constants are defined in its_constants)
# State keys
STATE = "STATE"
DEVICE = "DEVICE"
# State values added for RFT
#STATE_L6
#STATE_L7
STATE_DEVICE1 = "0"
STATE_DEVICE2 = "1"
#STATE_DEVICE3 = "3"
#STATE_DEVICE4 = "4"

BATTERY = "BATTERY_LEVEL"
GEN_VOLTAGE_L1 = "GENERATOR_L1"
GEN_VOLTAGE_L2 = "GENERATOR_L2"
GEN_VOLTAGE_L3 = "GENERATOR_L3"
MAINS_VOLTAGE_L1 = "MAINS_L1"
MAINS_VOLTAGE_L2 = "MAINS_L2"
MAINS_VOLTAGE_L3 = "MAINS_L3"
# Alarm key
ALARM = "ALARM"
# Alarm values
ALARM_OC = "Oc"
ALARM_UC = "Uc"
ALARM_TC = "Tc"


# Symbolic constants for Errors
# Error key
ERROR = "ERROR"
# Error values
ERROR_MANUAL = "Equipment is in manual control"
ERROR_DEVICE = "Error from device"
# ToDo add more specific errors

# Internal class specific for Siemens Main PLc (Universal equipment classes are defined in its_constants)
# ToDo add device classes (strobe, air etc)
# DEAP_SEA_CLASSES = {
# DEVICE: {"states": (STATE_UNKNOWN, STATE_ON, STATE_ON),
#              "controllable": True,
#              "device" : (STATE_DEVICE1, STATE_DEVICE2),
#              "description": "Generator power and device state",
#              },
#
# }

class DeapSea_8620:

    # This initialization defines members and their datatypes would like to have it here and not in init function but it does not work

    # _client= None # We could be using another Modbusclient type for serial for example, therefore not explicit definition here
    #  _device = 0 # device no of devices controlled by
    # _state = STATE_OFF # string id for light which is the state returned from the equipment
    # _status = "" # String Id for rft-status which is the status of the communication with the equipment
    # _detail = "" # Dict with human readable state, error and alarm {STATE: , DEVICE, ALARM:alarm_1+ alarm_2, ERROR:error_message)

    # Set the default parameters for the RTF as used in this context. Can be overwritten by inherited classes
    #_register_address_discrete = 40001 # Starting address of discrete outputs, only to be read, is set in holding registers
    #_no_status_registers_bits = 16 # No of status bits we want from rft (up to 128 including values)
    #_register_address_holding = 41025 # Starting address for holding registers

    def __init__(self, ip_address, port, device=0, timeout=0.5):
        """Initialisation of the class """
        # This initialization defines members and their datatypes.
        self._device = device
        self._state = STATE_OFF
        self._status = STATUS_FAILURE
        self._detail = {DEVICE:"", STATE:"", ALARM:"", ERROR:"", BATTERY:"", GEN_VOLTAGE_L1:"", GEN_VOLTAGE_L2:"",
                        GEN_VOLTAGE_L3:"", MAINS_VOLTAGE_L1:"", MAINS_VOLTAGE_L2:"", MAINS_VOLTAGE_L3:"",}  # No Alarm, No Error at start

        # Registers are used for both read statuses and write commands
        self._register_address_read_state = 768  # Start of status registers (page 3, 3*256)
        self._register_address_read_parameters = 1024 # Start of status registers (page 4, 4*256)
        self._register_address_digital_output = 3328  # Start of status registers (page 13, 13*256)
        self._register_address_write = 4096 # Start of command registers (Page 16, 16*256)

        self._register_address_read_state_offset = 4  # Reading starts at address 1024 for now, might change
        self._register_address_read_parameter_offset = 0 # Reading starts at address 1024 for now, might change
        self._register_address_read_digital_output_offset = 0  # Reading starts at address 1024 for now, might change
        self._register_address_write_offset = 6 # Command starts at address 4104
        self._no_read_registers_state_bytes = 2  # registers holding command state (auto, manual, stop)
        self._no_read_registers_parameter_bytes = 10  # registers holding parameters like battery voltage, main viltage oil level etc
        self._no_read_registers_digital_output_bytes = 4  # registers holding parameters like battery voltage, main viltage oil level etc
        self._no_write_registers_bytes = 2  # 2 bytes per write command

        # Starting address of holding register for read for device
        self._register_address_read_state_start = self._register_address_read_state + self._register_address_read_state_offset
        self._register_address_read_parameters_start = self._register_address_read_parameters + self._register_address_read_prameters_offset
        # Starting address of holding register for write for device
        self._register_address_write_command = self._register_address_write + self._register_address_write_offset

        # try: Modbus py will throw Modbus exception with commmunication error
        # We could catch and rethrow with updated status info, not done now
        self._client = ModbusTcpClient(ip_address, port, frame=ModbusSocketFramer, timeout=timeout)
        # except Exception as e: will be cought by encapsulating env
        #self._state = STATE_FAIL  # equipment not ready
        #self._status = STATUS_FAILURE
        #self._detail[ERROR] = str(e)

    def get_state(self):
        """Gets the light state for the set device/direction  and returns it as tuple with detail dict"""
        try:
            # Result is returned as integer value(?)
            # result = self._client.read_coils(self._register_address_discrete, self._no_status_registers_bits)
            # We assume that the Siemens
            result = self._client.read_holding_registers(self._register_address_read_state_start, self._no_read_registers_state_bytes)
            # print(f"register 1: {result.getRegister(0)}")
            # For debug
            # Todo replace with log
            #print(result.getRegister(0))
            #print(int(result.bits[0] == True))

            # First we read out the equipment state

            if result.getRegister(1) != 3: #3 is manual ? # See manual 056-079 Gencomm Status
                self._state = result.getRegister(5)
                self._status = STATUS_OK
            else
                # Treat manual as fail for status as we want equipment to be in remote control
                self._state = STATE_FAIL # Manual is not OK
                self._status = STATUS_FAILURE
                self._detail[ERROR] = ERROR_DEVICE # Todo get error codes from RFT

            result = self._client.read_holding_registers(self._register_address_read_parameters_start,
                                                         self._no_read_registers_parameter_bytes)
            # Generator status
            battery_level = result.getRegister(5)
            self._detail[BATTERY] = battery_level/10.0

            self._detail[STATE] = self._state
            self._detail[DEVICE] = self._device

        except Exception as e:
            self._status = STATUS_FAILURE
            self._state = STATE_FAIL
            self._detail[ERROR] = ERROR

        return (self._status, self._device, self._state, self._detail)

    def set_state(self, cmd, reset=0):
        """Sets the light strength of the rft for the device/direction it as tuple with detail dict"""

        # We could do ugly bitshifting and operating but chose numpy as we can have different sizes and for readability
        cmd_code = np.uint16(0)
        if cmd == CMD_STBY:
            cmd_code = np.uint16(35701) # Set to AUTO
        elif cmd == CMD_START:
            cmd_code = np.uint16(35705) # Start "Manually"
        elif cmd == CMD_CONN:
            cmd_code = np.uint16(35708) # Transfer to Generator
        # Not needed now?
        #elif cmd == CMD_STOP:
        #    cmd_code = np.uint16(35700)

        try:
            # self._client.write_registers(self._register_address_holding_write, [level][reset])
            self._client.write_registers(self._register_address_write_command, [cmd_code,~cmd_code])
            # or if writing direct to discrete output not used here
            # client.write_coils(41025, [True,False])
            return self.get_state()
        except Exception as e:
            self._status = STATUS_FAILURE
            self._state = STATE_FAIL
            self._detail[ERROR] = ERROR

        return (self._status, self._device, self._state, self._detail)

# Only for test as individual component using BIVM ip
#deapSea = DeapSea_8620('10.36.90.xxx', 502, device=0, 0.5)
#deapSea = DeapSea_8620('192.168.1.100', 502, device=0, 0.5)
#print(deapSea.get_state())
#print(deapSea.set_state(CMD_STBY))
