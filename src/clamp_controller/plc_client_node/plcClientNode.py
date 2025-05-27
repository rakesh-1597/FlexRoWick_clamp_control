#!/usr/bin/env python3

import snap7
from snap7.util import get_bool
from snap7.util import set_bool
from snap7.types import *
import struct
import rospy
import yaml
import sys
import os
#import logging
from std_msgs.msg import Bool, Float32

"""
# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [%(asctime)s]: %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
"""

class PlcClient:
    def __init__(self):
        self.loadYaml()
        self.client_ = snap7.client.Client()
        self.ip_ = rospy.get_param("clamp_controller/plc_ip")
        self.rack_ = rospy.get_param("clamp_controller/plc_rack")
        self.slot_ = rospy.get_param("clamp_controller/plc_slot")
        self.clampCtrlDbNum_ = rospy.get_param("clamp_controller/plc_db_number")
        self.isCalibrated_ = False
        self.currentClampAngle_ = float(0.00)
        self.actualClampAngle_ = float(0.00)
        self.clampRotating_ = False
        rospy.init_node("plc_client_node")
        rospy.Subscriber(rospy.get_param("clamp_controller/topic_relative_rotation"), Float32, self.callbackRelRotation)
        rospy.Subscriber(rospy.get_param("clamp_controller/topic_calibration_command"), Bool, self.callbackCalibrationDone)
        rospy.Subscriber(rospy.get_param("clamp_controller/topic_absolute_rotation"), Float32, self.callbackAbsRotation)
        rospy.Subscriber(rospy.get_param("clamp_controller/topic_stop_rotation"), Bool, self.callbackStopRotation)
        rospy.Subscriber(rospy.get_param("clamp_controller/topic_clamp_init"), Bool, self.callbackClampInit)
        self.clamp_angle_pub = rospy.Publisher(rospy.get_param("clamp_controller/topic_current_clamp_angle"), Float32, queue_size = 10)
        self.timer = rospy.Timer(rospy.Duration(1), self.publishClampAngle)
    
    def loadYaml(self):
        """Loads parameters from a YAML file into the ROS Parameter Server"""
        cwd = os.path.dirname(os.path.abspath(__file__))

        # Define the relative path to the YAML file (relative to the workspace root)
        yaml_relative_path = "config/plc_client.yaml"
        
        # Construct the absolute path
        yaml_path = os.path.join(cwd, yaml_relative_path)        
        if not os.path.exists(yaml_path):
            rospy.logwarn(f"YAML file not found: {yaml_path}")
            return
        
        try:
            with open(yaml_path, "r") as file:
                params = yaml.safe_load(file)
                for key, value in params.items():
                    rospy.set_param(key, value)  # Load params into ROS parameter server
            rospy.loginfo("Loaded parameters from YAML file.")
        except Exception as e:
            rospy.logerr(f"Failed to load YAML file: {e}")

    def connect(self):
        try:
            self.client_.connect(self.ip_, self.rack_, self.slot_)
            if self.client_.get_connected():
                rospy.loginfo("Connected to PLC")
        except Exception as e:
            rospy.logerr(f"Failed to connect to PLC: {e}")
            sys.exit(1)

    def callbackRelRotation(self, msg:Float32):
        if(not self.isCalibrated_):
            rospy.loginfo("Calibration not done! Please follow these steps below")
            rospy.loginfo("1) calibarate the clamp to zero postion using the hydraulic pin")
            rospy.loginfo("2) When done, publish execute ros topic:  flexrowick/calibration_done_cmd")
        else:
            rospy.loginfo(f"Entered value:{msg.data}")
            try:
                self.write_bool(self.clampCtrlDbNum_,10,1,False)
            except Exception as e:
                rospy.loginfo(f"Error resetting StartPosRel bit: {e}")

            self.write_double(self.clampCtrlDbNum_,28,msg.data)
            rospy.sleep(0.5)    #the rotation happens on a false to true edge of bStartPos_relativ signal as per TIA 
            try:
                self.write_bool(self.clampCtrlDbNum_,10,1,True)
            except Exception as e:
                rospy.loginfo(f"Error setting StartPosRel bit: {e}")
            # wait until the rotation is completed
            #while(self.isClampBusy()):  
             #   pass
            try:
                self.write_bool(self.clampCtrlDbNum_,10,1,False)
            except Exception as e:
                rospy.loginfo(f"Error resetting StartPosRel bit: {e}")

    def callbackAbsRotation(self, msg:Float32):
        if(not self.isCalibrated_):
            rospy.loginfo("Calibration not done! Please follow these steps below")
            rospy.loginfo("1) calibarate the clamp to zero postion using the hydraulic pin")
            rospy.loginfo("2) When done, publish execute ros topic:  flexrowick/calibration_done_cmd")
        else :
            currClampPos = self.currentClampAngle_
            rospy.loginfo(f"Current clamp pos:{currClampPos}")
            rospy.loginfo(f"Will  move to :{msg.data} now...")
            clockwise_rotation = (msg.data - currClampPos) % 360
            counterclockwise_rotation = (currClampPos - msg.data) % 360
            moveToAngle = -counterclockwise_rotation if counterclockwise_rotation < clockwise_rotation else clockwise_rotation
            try:
                self.write_bool(self.clampCtrlDbNum_,10,1,False)
            except Exception as e:
                rospy.loginfo(f"Error resetting StartPosRel bit: {e}")

            self.write_double(self.clampCtrlDbNum_, 28, moveToAngle)
            rospy.sleep(0.5)    #the rotation happens on a false to true edge of bStartPos_relativ signal as per TIA 

            try:
                self.write_bool(self.clampCtrlDbNum_,10,1,True)
            except Exception as e:
                rospy.loginfo(f"Error setting StartPosRel bit: {e}")

            # wait until the rotation is completed
            #while(self.isClampBusy()):
             #   rospy.sleep(0.5)  
              #  pass
            try:
                self.write_bool(self.clampCtrlDbNum_,10,1,False)
            except Exception as e:
                rospy.loginfo(f"Error resetting StartPosRel bit: {e}")

    def callbackCalibrationDone(self, msg:Bool):
        self.isCalibrated_ = bool(msg)
        if(self.isCalibrated_):
            self.actualClampAngle_ = self.readClampPosition()
        rospy.loginfo(f"Calibration flag set to {msg}  and actualClampAngle_ = {self.actualClampAngle_}")

    def callbackStopRotation(self, msg:Bool):
        
        rospy.loginfo(f"Stopping the rotation")
        try:
            self.write_bool(self.clampCtrlDbNum_,10,2, False) # TODO: Check with HMI panel on how to stop ongoing rotation
        except Exception as e:
            rospy.loginfo(f"Error in stopping rotation : {e}")
        rospy.sleep(0.2)
        try:
            self.write_bool(self.clampCtrlDbNum_,10,2, True) # TODO: Check with HMI panel on how to stop ongoing rotation
        except Exception as e:
            rospy.loginfo(f"Error in stopping rotation : {e}")

    
    def callbackClampInit(self, msg:Bool):
        rospy.loginfo(f"Initialising the clamp..")
        try:
            rospy.loginfo(f"Activating infeed now..")
            self.write_bool(self.clampCtrlDbNum_,10, 5,bool(msg))
            rospy.sleep(1)
            rospy.loginfo(f"Infeed Activated..")
        except Exception as e:
            rospy.loginfo(f"Error activating precharging: {e}")
        try:
            rospy.loginfo(f"Activating Pre Charging now..")
            self.write_bool(self.clampCtrlDbNum_,10, 3,bool(msg))
            rospy.sleep(1)
            rospy.loginfo(f"Precharging Activated..")
        except Exception as e:
            rospy.loginfo(f"Error activating precharging: {e}")
        try:
            rospy.loginfo(f"Activating axis now..")
            self.write_bool(self.clampCtrlDbNum_,11, 2,bool(msg))
            rospy.sleep(1)
            rospy.loginfo(f"Axis Activated..")
        except Exception as e:
            rospy.loginfo(f"Error activating Axis: {e}")
        
        try:
            axisReady = self.read_bit(self.clampCtrlDbNum_, 68, 1)
        except Exception as e:
            rospy.loginfo(f"Error reading  AxisReady bit field: {e}")
        rospy.loginfo(f"AxisReady:{axisReady}")

        try:
            axisError = self.read_bit(self.clampCtrlDbNum_, 68, 3)
        except Exception as e:
            rospy.loginfo(f"Error reading  AxisError bit field: {e}")
        rospy.loginfo(f"AxisError:{axisError}")

        if((axisError == 1) and (axisReady == 0 )):
            try:
                self.write_bool(self.clampCtrlDbNum_,10, 4,bool(msg))
            except Exception as e:
                rospy.loginfo(f"Error writing  10.4 bit field: {e}")
    
    def disconnect(self):
        self.client_.disconnect()
        rospy.loginfo("Disconnected from PLC")

    def read_db_data(self, db_number, start, size):
        try:
            return self.client_.db_read(db_number, start, size)
        except Exception as e:
            rospy.logerr(f"Failed to read DB data: {e}")
            sys.exit(1)

    def readClampPosition(self) -> float:
        try:
            return struct.unpack('>d',self.read_db_data(self.clampCtrlDbNum_, 12, 8))[0]
        except Exception as e:
            rospy.logerr(f"Failed to read from DB{self.clampCtrlDbNum_} data at 12th byte: {e}")
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
        self.waitForPlc()
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
        self.waitForPlc()
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
        self.waitForPlc()
        try:
            # Read 1 byte from the PLC
            byte_value = self.client_.db_read(db_number, byte_offset, 1)[0]

            # Extract the specific bit
            bit_value = (byte_value >> bit_offset) & 1
            return bool(bit_value)
        except Exception as e:
            rospy.logerr(f"Failed to read bit from PLC at  DB{self.clampCtrlDbNum_} byte {byte_offset} bit{bit_offset}: {e}")
            return None

    def disconnect(self):
        if self.client_.get_connected():
            self.client_.disconnect()
            rospy.loginfo("Disconnected from PLC.")
    
    def isClampBusy(self) -> bool:
        return self.read_bit(self.clampCtrlDbNum_, 68, 4)
    
    def runNodeLoop(self):
        rospy.loginfo("Starting Plc Client Node")
        self.connect()
        rospy.loginfo(f"Plc Info: {self.client_.get_cpu_info()}")
        rospy.on_shutdown(self.disconnect)
        rospy.loginfo(f"Waiting for data...")
        rospy.spin()

    def publishClampAngle(self, timer):
        if(self.isCalibrated_):
            self.waitForPlc()
            self.currentClampAngle_ = round((self.readClampPosition() - self.actualClampAngle_) % 360.00,3)
            self.clamp_angle_pub.publish(self.currentClampAngle_ )
            rospy.loginfo(f"published clamp angle:{self.currentClampAngle_}")
        else:
            rospy.loginfo(f"calibration not done yet or rotation is being executed")
    
    def waitForPlc(self):
        """Wait until the PLC is ready to accept new jobs."""
        while True:
            try:
                status = self.client_.get_cpu_state()
                #rospy.loginfo(f"waiting for plc, status:{status}")
                if status == "S7CpuStatusRun":
                    return  # PLC is ready
            except Exception as e:
                rospy.logwarn(f"Waiting for PLC to become ready: {e}")
            rospy.sleep(0.5)  # Wait before checking again
        
    

if __name__ == "__main__":
    plc = PlcClient()
    rospy.loginfo("Starting Plc Client Node")
    plc.connect()
    rospy.loginfo(f"Plc Info: {plc.client_.get_cpu_info()}")
    rospy.on_shutdown(plc.disconnect)
    rospy.loginfo(f"Waiting for data...")
    rospy.spin()


