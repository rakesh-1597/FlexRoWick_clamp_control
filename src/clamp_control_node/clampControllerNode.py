#!/usr/bin/env python3

import snap7
from snap7.util import get_bool
from snap7.util import set_bool
from snap7.types import *
import rospy
import sys
import struct
from builtins import open
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [%(asctime)s]: %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)


class PlcClient:
    def __init__(self, ip, rack, slot):
        self.client = snap7.client.Client()
        self.ip = ip
        self.rack = rack
        self.slot = slot

    def connect(self):
        try:
            self.client.connect(self.ip, self.rack, self.slot)
            if self.client.get_connected():
                rospy.loginfo("Connected to PLC")
        except Exception as e:
            rospy.logerr(f"Failed to connect to PLC: {e}")
            sys.exit(1)

    def disconnect(self):
        self.client.disconnect()
        rospy.loginfo("Disconnected from PLC")

    def read_db_data(self, db_number, start, size):
        try:
            return self.client.db_read(db_number, start, size)
        except Exception as e:
            rospy.logerr(f"Failed to read DB data: {e}")
            sys.exit(1)

    def write_db_data(self, db_number, start, data):
        try:
            self.client.db_write(db_number, start, data)
            rospy.loginfo(f"Successfully wrote data to DB {db_number} at start {start}")
        except Exception as e:
            rospy.logerr(f"Failed to write DB data: {e}")
            sys.exit(1)

    def is_connected(self):
        return self.client.get_connected()
    
    def list_blocks(self, ip):
        try:
            self.connect()
            rospy.loginfo(f"Connected to PLC at {ip}")
            
            blocks = self.client.list_blocks()
            rospy.loginfo("Available Blocks:")
            rospy.loginfo(f"  OBs: {blocks['OB']}")
            rospy.loginfo(f"  FBs: {blocks['FB']}")
            rospy.loginfo(f"  FCs: {blocks['FC']}")
            rospy.loginfo(f"  DBs: {blocks['DB']}")
            rospy.loginfo(f"  SDBs: {blocks['SDB']}")
            
        except Exception as e:
            rospy.loginfo(f"Error: {e}")
        finally:
            if self.client.get_connected():
                self.client.disconnect()
                rospy.loginfo("Disconnected from PLC")
if __name__ == "__main__":
    rospy.init_node("clamp_controller_node")
    rospy.loginfo("Starting Clamp Controller Node")
    plc = PlcClient("172.31.1.160", 0, 1)
    #plc.list_blocks("172.31.1.160")
    plc.connect()
    print(plc.client.get_cpu_info())
    jogForward = False
    while True:
        data =plc.client.db_read(2,10,1)
        jogForward = get_bool(data,0,7)
        print(f"bJogForward:{jogForward}")
        newVal = not jogForward
        print(f"setting JogForwardto:{newVal}")
        set_bool(data,0,7,newVal)
        try:
            plc.client.db_write(2,10,data)
        except Exception as e:
            print(f"Error: {e}")
        rospy.sleep(5)
    
    #plc.list_blocks(plc.ip)
    rospy.loginfo("sleeping")
    rospy.sleep(10)
    '''
    if len(sys.argv) != 2:
        rospy.logerr("Usage: rosrun clamp_control_node clampControllerNode.py <float_value>")
        sys.exit(1)

    try:
        value = float(sys.argv[1])
    except ValueError:
        rospy.logerr("The argument must be a float.")
        sys.exit(1)

    plc = PlcClient("197.168.100", 0, 2)
    plc.connect()
    rospy.loginfo("PLC connected")

    # Assuming the float value needs to be written to DB number 1, starting at byte 0
    db_number = 1
    start = 0
    data = bytearray(struct.pack("f", value))
    plc.write_db_data(db_number, start, data)
    rospy.loginfo(f"Written value {value} to DB {db_number} at start {start}")

    plc.disconnect()'''
    plc.disconnect()
    rospy.loginfo("PLC disconnected")
