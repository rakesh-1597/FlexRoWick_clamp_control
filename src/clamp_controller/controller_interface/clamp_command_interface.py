#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Bool

class ClampCmdInterface:
    def __init__(self):
        rospy.init_node("clamp_command_interface")
        self.activateInfeedCommandPub_ = rospy.Publisher("flexrowick/activate_infeed", Bool, queue_size= 1)
        self.activatePrechargingCommandPub_ = rospy.Publisher("flexrowick/activate_precharging", Bool, queue_size= 1)
        self.relativeRotationPub_ = rospy.Publisher("flexrowick/clamp_relative_rotation", Float32, queue_size = 10)
        self.absoluteRotationPub_ = rospy.Publisher("flexrowick/clamp_absolute_rotation", Float32, queue_size= 10)
        self.calibrationCommandPub_ = rospy.Publisher("flexrowick/calibration_done_cmd", Bool, queue_size= 10)
        self.stopRotationPub_ = rospy.Publisher("flexrowick/stop_rotation", Bool, queue_size= 1)
        self.initialiseClampPub_ = rospy.Publisher("flexrowick/clamp_init", Bool, queue_size= 1)
        rospy.sleep(1)  #give some time for rosNode to register the publishers for 1 second

    def relRotation(self, angle:float):
        try:
            relative_rotation = angle
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Float:{e}")
        try:
            self.relativeRotationPub_.publish(Float32(data=float(relative_rotation)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish relative rotation value!")

    def absRotation(self, angle:float):
        try:
            absolute_rotation = angle
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Float:{e}")
        try:
            self.absoluteRotationPub_.publish(Float32(data=float(absolute_rotation)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish absolute rotation value!")
    
    def setCalibration(self, calibrationFlag:bool):
        try:
            calibFlag = calibrationFlag
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Bool:{e}")
        try:
            self.calibrationCommandPub_.publish(Bool(data=bool(calibFlag)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish calibration command flag!")
    
    def stopRotation(self, stopFlag:bool):
        try:
            flag = stopFlag
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Bool:{e}")
        try:
            self.stopRotationPub_.publish(Bool(data=bool(flag)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish stop rotation command flag!")

    def activateInfeed(self, actFlag:bool):
        try:
            flag = actFlag
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Bool:{e}")
        try:
            self.activateInfeedCommandPub_.publish(Bool(data=bool(flag)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish activate Infeed command flag!")
    
    def activatePrecharging(self, actFlag:bool):
        try:
            flag = actFlag
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Bool:{e}")
        try:
            self.activatePrechargingCommandPub_.publish(Bool(data=bool(flag)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish activate Precharging command flag!")

    def initialiseClamp(self, initFlag:bool):
        try:
            flag = initFlag
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Bool:{e}")
        try:
            self.initialiseClampPub_.publish(Bool(data=bool(flag)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish initialise clamp command flag!")