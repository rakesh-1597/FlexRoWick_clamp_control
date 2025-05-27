#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Bool
import os
import yaml

class ClampCmdInterface:
    def __init__(self):
        self.loadYaml()
        rospy.init_node("clamp_command_interface")
        self.relativeRotationPub_ = rospy.Publisher(rospy.get_param("clamp_controller/topic_relative_rotation"), Float32, queue_size = 10)
        self.absoluteRotationPub_ = rospy.Publisher(rospy.get_param("clamp_controller/topic_absolute_rotation"), Float32, queue_size= 10)
        self.calibrationCommandPub_ = rospy.Publisher(rospy.get_param("clamp_controller/topic_calibration_command"), Bool, queue_size= 10)
        self.stopRotationPub_ = rospy.Publisher(rospy.get_param("clamp_controller/topic_stop_rotation"), Bool, queue_size= 1)
        self.initialiseClampPub_ = rospy.Publisher(rospy.get_param("clamp_controller/topic_clamp_init"), Bool, queue_size= 1)
        rospy.sleep(1)  #give some time for rosNode to register the publishers for 1 second
    def loadYaml(self):
        """Loads parameters from a YAML file into the ROS Parameter Server"""
        cwd = os.path.dirname(os.path.abspath(__file__))

        # Define the relative path to the YAML file (relative to the workspace root)
        yaml_relative_path = "config/clamp_controller.yaml"
        
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

    def initialiseClamp(self, initFlag:bool):
        try:
            flag = initFlag
        except ValueError as e:
            rospy.loginfo(f"Error! Entered value is not of type Bool:{e}")
        try:
            self.initialiseClampPub_.publish(Bool(data=bool(flag)))
        except rospy.ROSException:
            rospy.loginfo("Failed to publish initialise clamp command flag!")