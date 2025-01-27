import snap7
from snap7.util import *
from snap7.types import *
import rospy
import sys
import struct

class PlcClient:
    def __init__(self, ip, rack, slot):
        self.client = snap7.client.Client()
        self.ip = ip
        self.rack = rack
        self.slot = slot

    def connect(self):
        try:
            self.client.connect(self.ip, self.rack, self.slot)
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
    
if __name__ == "__main__":
    rospy.init_node("clamp_controller_node")
    rospy.loginfo("Starting Clamp Controller Node")
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

    plc.disconnect()
    rospy.loginfo("PLC disconnected")
