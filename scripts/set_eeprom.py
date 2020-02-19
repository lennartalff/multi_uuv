#!/usr/bin/env python
"""
This is a convinience module to write eeprom settings to the rf module.

Usage:
set_eeprom <nodeid>
"""
from __future__ import print_function
from __future__ import division
import time
import sys
import re
import serial

EEPROM_PARAMETERS = dict(
    serial_speed=57,
    air_speed=64,
    netid=5,
    txpower=11,
    mavlink=1,
    oppresend=0,
    min_freq=433050,
    max_freq=434790,
    num_channels=10,
    duty_cycle=100,
    lbt_rssi=0,
    manchester=0,
    rtscts=0,
    nodeid=0,  # overwritten by command line argument
    nodedestination=65535,  # 65535 means broadcasting
    syncany=0,
    nodecount=3,
)

AT_EEPROM = dict(
    serial_speed=b"ATS1",
    air_speed=b"ATS2",
    netid=b"ATS3",
    txpower=b"ATS4",
    mavlink=b"ATS6",
    oppresend=b"ATS7",
    min_freq=b"ATS8",
    max_freq=b"ATS9",
    num_channels=b"ATS10",
    duty_cycle=b"ATS11",
    lbt_rssi=b"ATS12",
    manchester=b"ATS13",
    rtscts=b"ATS14",
    nodeid=b"ATS15",
    nodedestination=b"ATS16",
    syncany=b"ATS17",
    nodecount=b"ATS18"
)

AT_COMMAND = dict(
    write_eeprom=b"AT&W\r\n",
    exit=b"ATO\r\n",
    reboot=b"ATZ\r\n",
    show_eeprom=b"ATI5\r\n"

)


def enter_at_mode(serial_connection):
    """Entering the at mode of the module to allow configuration changes.

    :param serial_connection: Reference to the serial object.
    """
    while True:
        try:
            serial_connection.write(b"\r\n")
            serial_connection.write(AT_COMMAND["exit"])
            time.sleep(1.5)
            serial_connection.write(b"+++")
            while True:
                line = serial_connection.readline()
                print(line)
                if b"OK" in line:
                    break
                if line == "":
                    raise serial.SerialException()
        except serial.SerialException:
            print("Could not enter serial mode. Keep trying...")
            time.sleep(1.5)
        else:
            break


def set_eeprom_param(serial_connection, address, value):
    """Sets a specified EEPROM parameter.

    :param serial_connection: Reference to the serial object.
    :param address: Address of the parameter that should be set.
    :param value: Parameter value that is written.
    """
    serial_connection.write(b"{}={}\r\n".format(address, value))
    while True:
        line = serial_connection.readline()
        if b"OK" in line:
            break
        if line == b"":
            raise serial.SerialException()


def read_parameters_from_eeprom(serial_connection):
    """Reads the currently set parameters.

    :param serial_connection: Reference to the serial object.
    :return: Returns a dictionary of the parameters.
    """
    serial_connection.write(AT_COMMAND["show_eeprom"])
    parameters = {}
    while True:
        answer = serial_connection.readline()
        if answer == "":
            break
        match = re.search(b"S[0-9]*: ([a-zA-z]*)=([0-9]*)",
                          answer, re.IGNORECASE)
        if match is not None:
            parameters[match.group(1).lower()] = int(match.group(2))
    return parameters


def write_parameters_to_eeprom(serial_connection,
                               current_parameters,
                               desired_parameters):
    """Writes a set of parameters. The parameters are not persistent until
    they are written to the EEPROM.
    :param serial_connection: Reference to the serial object.
    :param current_parameters: Parameters that are currently active.
    :param desired_parameters: Parameters that are to be written.
    """
    if current_parameters["nodeid"] < desired_parameters["nodecount"]:
        set_eeprom_param(
            serial_connection,
            AT_EEPROM["nodecount"],
            desired_parameters["nodecount"])
    else:
        set_eeprom_param(
            serial_connection,
            AT_EEPROM["nodeid"],
            desired_parameters["nodeid"])
    for param in desired_parameters:
        set_eeprom_param(
            serial_connection,
            AT_EEPROM[param],
            desired_parameters[param])


def check_parameters(params1, params2):
    """Checks if parameter of the input arguments are the same.

    :param params1:
    :param params2:
    :return: Returns true if all parameters of params1 are in params2 and
    do not differ.
    """
    success = True
    try:
        for param in params1:
            if params1[param] != params2[param]:
                print("{}: '{}' != '{}'".format(
                    param, params1[param],
                    params2[param]))
                success = False
    except KeyError as err:
        print(err)
        return False
    else:
        return success


def create_serial_connection(port, baud, timeout=None):
    """Creates a serial connection. Keeps trying if connection could not be
    created.

    :param port: Port of the serial connection
    :param baud: Baud rate.
    :param timeout: Optional timeout parameter.
    :return: Returns reference of the serial object.
    """
    while True:
        try:
            serial_connection = serial.Serial(port, baud, timeout=timeout)
        except serial.SerialException as err:
            print("{}\nKeep trying...".format(err))
            time.sleep(1.0)
        else:
            break
    return serial_connection


def main(_nodeid, _port):
    """Main function of this module.

    :param _nodeid: The nodeid is provided via command line argument.
    :return: Boolean value of success.
    """
    eeprom_parameters = EEPROM_PARAMETERS
    eeprom_parameters["nodeid"] = int(_nodeid)
    if _port is None:
        _port = "/dev/ttyUSB0"
    serial_connection = create_serial_connection(
        _port,
        57600,
        timeout=3)
    print("Serial connection established")
    time.sleep(1.0)

    enter_at_mode(serial_connection)
    print("Entered AT mode")
    current_params = read_parameters_from_eeprom(serial_connection)
    if check_parameters(eeprom_parameters, current_params):
        print("Nothing needs to be changed")
    else:
        write_parameters_to_eeprom(serial_connection,
                                   current_params,
                                   eeprom_parameters)
        current_params = read_parameters_from_eeprom(serial_connection)
        if not check_parameters(eeprom_parameters, current_params):
            print("Could not set parameters correctly")
            serial_connection.close()
            return False

    print("NodeID: {}\n"
          "NodeCount: {}\n"
          "NetID: {}\n".format(current_params["nodeid"],
                               current_params["nodecount"],
                               current_params["netid"]))
    serial_connection.write(AT_COMMAND["write_eeprom"])
    serial_connection.write(AT_COMMAND["reboot"])
    serial_connection.close()
    return True


if __name__ == "__main__":
    try:
        NODE_ID = int(sys.argv[1])
    except IndexError:
        print("No Node ID specified. Usage: '{} <nodeid> [port]'".format(sys.argv[0]))
    else:
        PORT = None
        if len(sys.argv) >= 3:
            PORT = sys.argv[2]
        main(NODE_ID, PORT)
