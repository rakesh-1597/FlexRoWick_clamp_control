#!/usr/bin/env python3

import snap7
from snap7.util import get_bool
from snap7.util import set_bool
from snap7.types import *
import struct
import rospy
import sys
import logging
from std_msgs.msg import Float32MultiArray

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
        rospy.init_node("clamp_control_node")
        rospy.Subscriber("flexrowick/clamp_output_params", Float32MultiArray, self.callback)

    def connect(self):
        try:
            self.client.connect(self.ip, self.rack, self.slot)
            if self.client.get_connected():
                rospy.loginfo("Connected to PLC")
        except Exception as e:
            rospy.logerr(f"Failed to connect to PLC: {e}")
            sys.exit(1)

    def callback(self, msg:Float32MultiArray):
        rospy.loginfo(f"Entered value:{msg.data[0]}")
        try:
            plc.write_bool(2,10,1,False)
        except Exception as e:
            rospy.loginfo(f"Error resetting StartPosRel bit: {e}")
        plc.write_double(2,28,msg.data[0])
        try:
            plc.write_bool(2,10,1,True)
        except Exception as e:
            rospy.loginfo(f"Error setting StartPosRel bit: {e}")
        rospy.sleep(2)
        try:
            plc.write_bool(2,10,1,False)
        except Exception as e:
            rospy.loginfo(f"Error resetting StartPosRel bit: {e}")
        #newVal = not jogForward
        #print(f"setting JogForwardto:{newVal}")
        #set_bool(data,0,7,newVal)
        rospy.sleep(5)

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
    
    def write_double(self, db_number, start, value):
        """ Write an 8-byte float (double) value to a specific DB address """
        try:
            # Convert double (8-byte float) to bytes (Big-endian format for Siemens)
            data = bytearray(struct.pack(">d", value))
            self.client.db_write(db_number, start, data)
            rospy.loginfo(f"Written {value} to DB {db_number} at start {start}")
        except Exception as e:
            rospy.logerr(f"Failed to write to PLC: {e}")
    
    def write_bool(self, db_number, byte_offset, bit_offset, value):
        """
        Write a boolean value to a specific bit in a DB byte.
        
        :param db_number: The DB number (e.g., DB1)
        :param byte_offset: The byte address (e.g., 10)
        :param bit_offset: The bit within the byte (0-7, e.g., 1 for 10.1)
        :param value: The boolean value (True or False)
        """
        try:
            # Read the current byte from the PLC
            current_byte = self.client.db_read(db_number, byte_offset, 1)[0]
            
            # Modify the specific bit
            if value:
                # Set the bit (OR operation)
                modified_byte = current_byte | (1 << bit_offset)
            else:
                # Clear the bit (AND with inverted bit mask)
                modified_byte = current_byte & ~(1 << bit_offset)
            
            # Write the modified byte back to the PLC
            self.client.db_write(db_number, byte_offset, bytes([modified_byte]))
            rospy.loginfo(f"Written BOOL {value} to DB {db_number} at {byte_offset}.{bit_offset}")
        except Exception as e:
            rospy.logerr(f"Failed to write BOOL to PLC: {e}")

    def disconnect(self):
        if self.client.get_connected():
            self.client.disconnect()
            rospy.loginfo("Disconnected from PLC.")
    
if __name__ == "__main__":
    plc = PlcClient("172.31.1.160", 0, 1)
    rospy.loginfo("Starting Clamp Controller Node")
    plc.connect()
    rospy.loginfo(f"Plc Info: {plc.client.get_cpu_info()}")
    rospy.on_shutdown(plc.disconnect)
    rospy.loginfo(f"Waiting for data...")
    rospy.spin()
    """while True:
        rotationVal = float(input("Enter the rotation degree value"))
        data =plc.client.db_read(2,10,1)
        jogForward = get_bool(data,0,7)
        print(f"Entered value:{rotationVal}")
        try:
            plc.write_bool(2,10,1,False)
        except Exception as e:
            print(f"Error resetting StartPosRel bit: {e}")
        plc.write_double(2,28,rotationVal)
        try:
            plc.write_bool(2,10,1,True)
        except Exception as e:
            print(f"Error setting StartPosRel bit: {e}")
        rospy.sleep(2)
        try:
            plc.write_bool(2,10,1,False)
        except Exception as e:
            print(f"Error resetting StartPosRel bit: {e}")
        #newVal = not jogForward
        #print(f"setting JogForwardto:{newVal}")
        #set_bool(data,0,7,newVal)
        rospy.sleep(5)
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

    plc.disconnect()"""
