#!/usr/bin/env python3

import rospy
import threading
import os
from std_msgs.msg import Float32MultiArray, Bool
from plc_client_node.plcClientNode import PlcClient
from controller_interface.clamp_command_interface import ClampCmdInterface
import yaml

def relRot(angle):
    try:
        relative_rotation = angle
    except ValueError as e:
        print(f"Error! Entered value is not of type Float:{e}")
    try:
        tpub_relative_rotation.publish(Float32MultiArray(data=[float(relative_rotation)]))
    except rospy.ROSException:
        print("Failed to publish relative rotation value!")

def userInputLoop(interfaceObj:ClampCmdInterface):
    while True:
        print("Welcome to the FlexRoWick Clamp Controller terminal!")
        print("Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration")
        print("Enter the command number:")
        try:
            command = int(input("Enter the command number: "))
        except ValueError:
            print("Invalid input! Please enter a number.")
            continue
        if command == 1:
            try:
                relative_rotation = input("Enter the relative rotation value: ")
            except ValueError as e:
                print(f"Error! Entered value is not of type Float {e}")
            interfaceObj.relRotation(relative_rotation)
        elif command == 2:
            try:
                absolute_rotation = input("Enter the absolute rotation value: ")
            except ValueError as e:
                print(f"Error! Entered value is not of type Float {e}")            
            interfaceObj.absRotation(absolute_rotation)
        elif command == 3:
            try:
                flag = int(input("Enter calibration Done Flag: 1(True) or 0(False)"))
            except ValueError as e:
                print(f"Error! Entered value is not of type int {e}")
            interfaceObj.setCalibration(bool(flag))
        else:
            print("Invalid command! Please enter a valid command number.")
def loadYaml():
    """Loads parameters from a YAML file into the ROS Parameter Server"""
    cwd = os.getcwd()

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
        print("Loaded parameters from YAML file.")
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {e}")

if __name__ == "__main__":
    clampCmdIf = ClampCmdInterface()
    userInputLoop(clampCmdIf)


    