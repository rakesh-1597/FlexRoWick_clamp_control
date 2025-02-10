#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Bool

class ClampCmdInterface:
    def __init__(self):
        rospy.init_node("clamp_command_interface")
        self.relativeRotationPub_ = rospy.Publisher("flexrowick/clamp_relative_rotation", Float32MultiArray, queue_size = 10)
        self.absoluteRotationPub_ = rospy.Publisher("flexrowick/clamp_absolute_rotation", Float32MultiArray, queue_size= 10)
        self.calibrationCommandPub_ = rospy.Publisher("flexrowick/calibration_done_cmd", Bool, queue_size= 10)
        rospy.sleep(1)  #give some time for rosNode to register the publishers

    def relRotation(self, angle:float):
        try:
            relative_rotation = angle
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Float:{e}")
        try:
            self.relativeRotationPub_.publish(Float32MultiArray(data=[float(relative_rotation)]))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish relative rotation value!")

    def absRotation(self, angle:float):
        try:
            absolute_rotation = angle
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Float:{e}")
        try:
            self.absoluteRotationPub_.publish(Float32MultiArray(data=[float(absolute_rotation)]))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish absolute rotation value!")
    
    def setCalibration(self, calibrationFlag:bool):
        try:
            calibFlag = calibrationFlag
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Float:{e}")
        try:
            self.calibrationCommandPub_.publish(Bool(data=bool(calibFlag)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish calibration command flag!")