#!/usr/bin/env python3

import snap7
from snap7.util import get_bool
from snap7.util import set_bool
from snap7.types import *
import struct
import rospy
import sys
import logging
from std_msgs.msg import Float32MultiArray, Bool

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
        self.client_ = snap7.client.Client()
        self.ip_ = ip
        self.rack_ = rack
        self.slot_ = slot
        self.clampCtrlDb_ = 2
        self.isCalibrated = False
        self.currentClampAngle = float(0.00)
        rospy.init_node("clamp_control_node")
        rospy.Subscriber("flexrowick/clamp_relative_rotation", Float32MultiArray, self.callbackRelRotation)
        rospy.Subscriber("flexrowick/calibration_done_cmd", Bool, self.callbackCalibrationDone)
        rospy.Subscriber("flexrowick/clamp_absolute_rotation", Float32MultiArray, self.callbackAbsRotation)

    def connect(self):
        try:
            self.client_.connect(self.ip_, self.rack_, self.slot_)
            if self.client_.get_connected():
                rospy.loginfo("Connected to PLC")
        except Exception as e:
            rospy.logerr(f"Failed to connect to PLC: {e}")
            sys.exit(1)

    def callbackRelRotation(self, msg:Float32MultiArray):
        if(not self.isCalibrated):
            rospy.loginfo("Calibration not done! Please follow these steps below")
            rospy.loginfo("1) calibarate the clamp to zero postion using the hydraulic pin")
            rospy.loginfo("2) When done, publish execute ros topic:  flexrowick/calibration_done_cmd")
        else:
            rospy.loginfo(f"Entered value:{msg.data[0]}")
            try:
                plc.write_bool(self.clampCtrlDb_,10,1,False)
            except Exception as e:
                rospy.loginfo(f"Error resetting StartPosRel bit: {e}")

            plc.write_double(self.clampCtrlDb_,28,msg.data[0])
            rospy.sleep(0.5)    #the rotation happens on a false to true edge of bStartPos_relativ signal as per TIA 

            try:
                plc.write_bool(self.clampCtrlDb_,10,1,True)
            except Exception as e:
                rospy.loginfo(f"Error setting StartPosRel bit: {e}")
            rospy.sleep(1)
            # wait until the rotation is completed
            while(self.isClampBusy()):  
                pass
            self.currentClampAngle = (self.currentClampAngle + msg.data[0]) % 360.0
            rospy.loginfo(f"current clamp angle(program evaluated): {self.currentClampAngle}")
            try:
                plc.write_bool(self.clampCtrlDb_,10,1,False)
            except Exception as e:
                rospy.loginfo(f"Error resetting StartPosRel bit: {e}")  

    def callbackAbsRotation(self, msg:Float32MultiArray):
        if(not self.isCalibrated):
            rospy.loginfo("Calibration not done! Please follow these steps below")
            rospy.loginfo("1) calibarate the clamp to zero postion using the hydraulic pin")
            rospy.loginfo("2) When done, publish execute ros topic:  flexrowick/calibration_done_cmd")
        else :
            currClampPos = self.currentClampAngle
            rospy.loginfo(f"Current clamp pos:{currClampPos}")
            rospy.loginfo(f"Will  move to :{msg.data[0]} now...")
            moveToAngle = float((msg.data[0] - currClampPos) % 360.0)
            try:
                plc.write_bool(self.clampCtrlDb_,10,1,False)
            except Exception as e:
                rospy.loginfo(f"Error resetting StartPosRel bit: {e}")

            plc.write_double(self.clampCtrlDb_, 28, moveToAngle)
            rospy.sleep(0.5)    #the rotation happens on a false to true edge of bStartPos_relativ signal as per TIA 

            try:
                plc.write_bool(self.clampCtrlDb_,10,1,True)
            except Exception as e:
                rospy.loginfo(f"Error setting StartPosRel bit: {e}")
            rospy.sleep(1)

            # wait until the rotation is completed
            while(self.isClampBusy()):  
                pass
            self.currentClampAngle = float(msg.data[0] %  360.0)
            try:
                plc.write_bool(self.clampCtrlDb_,10,1,False)
            except Exception as e:
                rospy.loginfo(f"Error resetting StartPosRel bit: {e}")
            rospy.loginfo(f"current clamp angle(program evaluated): {self.currentClampAngle}")

    def callbackCalibrationDone(self, msg:Bool):
        self.isCalibrated = bool(msg)
        self.currentClampAngle = float(0.0)
        rospy.loginfo(f"Calibration flag set to {msg}  and currentClampAngle set to 0.0 !")
    
    def disconnect(self):
        self.client_.disconnect()
        rospy.loginfo("Disconnected from PLC")

    def read_db_data(self, db_number, start, size):
        try:
            return self.client_.db_read(db_number, start, size)
        except Exception as e:
            rospy.logerr(f"Failed to read DB data: {e}")
            sys.exit(1)
    def read_current_position(self) -> float:
        try:
            return struct.unpack('>d',self.read_db_data(self.clampCtrlDb_, 12, 8))[0]
        except Exception as e:
            rospy.logerr(f"Failed to read from DB{self.clampCtrlDb_} data at 12th byte: {e}")
            sys.exit(1)
    def write_db_data(self, db_number, start, data):
        try:
            self.client_.db_write(db_number, start, data)
            rospy.loginfo(f"Successfully wrote data to DB {db_number} at start {start}")
        except Exception as e:
            rospy.logerr(f"Failed to write DB data: {e}")
            sys.exit(1)

    def is_connected(self):
        return self.client_.get_connected()
    
    def write_double(self, db_number, start, value):
        """ Write an 8-byte float (double) value to a specific DB address """
        try:
            # Convert double (8-byte float) to bytes (Big-endian format for Siemens)
            data = bytearray(struct.pack(">d", value))
            self.client_.db_write(db_number, start, data)
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
            current_byte = self.client_.db_read(db_number, byte_offset, 1)[0]
            
            # Modify the specific bit
            if value:
                # Set the bit (OR operation)
                modified_byte = current_byte | (1 << bit_offset)
            else:
                # Clear the bit (AND with inverted bit mask)
                modified_byte = current_byte & ~(1 << bit_offset)
            
            # Write the modified byte back to the PLC
            self.client_.db_write(db_number, byte_offset, bytes([modified_byte]))
            rospy.loginfo(f"Written BOOL {value} to DB {db_number} at {byte_offset}.{bit_offset}")
        except Exception as e:
            rospy.logerr(f"Failed to write BOOL to PLC: {e}")

    def read_bit(self, db_number, byte_offset, bit_offset):
        """
        Reads a specific bit from a byte in a Data Block.
        :param db_number: Data Block number (e.g., 1 for DB1)
        :param byte_offset: Byte index (e.g., 10 for DBX10.x)
        :param bit_offset: Bit index within the byte (0-7)
        :return: Boolean value of the bit (True/False)
        """
        try:
            # Read 1 byte from the PLC
            byte_value = self.client_.db_read(db_number, byte_offset, 1)[0]

            # Extract the specific bit
            bit_value = (byte_value >> bit_offset) & 1
            return bool(bit_value)
        except Exception as e:
            rospy.logerr(f"Failed to read bit from PLC at  DB{self.clampCtrlDb_} byte {byte_offset} bit{bit_offset}: {e}")
            return None

    def disconnect(self):
        if self.client_.get_connected():
            self.client_.disconnect()
            rospy.loginfo("Disconnected from PLC.")
    
    def isClampBusy(self) -> bool:
        return self.read_bit(self.clampCtrlDb_, 68, 4)
    
if __name__ == "__main__":
    plc = PlcClient("172.31.1.160", 0, 1)
    rospy.loginfo("Starting Clamp Controller Node")
    plc.connect()
    rospy.loginfo(f"Plc Info: {plc.client_.get_cpu_info()}")
    rospy.on_shutdown(plc.disconnect)
    rospy.loginfo(f"Waiting for data...")
    rospy.spin()
