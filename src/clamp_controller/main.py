#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from plc_client_node.src.plc_client_node.plcClientNode import PlcClient
if __name__ == "__main__":
    plc = PlcClient()
    rospy.loginfo("Starting Plc Client Node")
    plc.connect()
    rospy.loginfo(f"Plc Info: {plc.client_.get_cpu_info()}")
    rospy.on_shutdown(plc.disconnect)
    rospy.loginfo(f"Waiting for data...")
    rospy.spin()
    tpub_relative_rotation = rospy.Publisher(rospy.get_param("/tpub_relative_rotation"), Float32MultiArray, queue_size=10)
    tpub_absolute_rotation = rospy.Publisher(rospy.get_param("/tpub_absolute_rotation"), Float32MultiArray, queue_size=10)
    tpub_calibration_done = rospy.Publisher(rospy.get_param("/tpub_calibration_command"), Bool, queue_size=10)
    while(True):
        print("Welcome to the FlexRoWick Clamp Controller terminal!")
        print("Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration")
        print("Enter the command number:")
        try:
            command = input("Enter the command number: ")
        except ValueError:
            print("Invalid input! Please enter a number.")
            continue
        if command == 1:
            relative_rotation = input("Enter the relative rotation value: ")
            try:
                tpub_relative_rotation.publish(Float32MultiArray(data=[relative_rotation]))
            except rospy.ROSException:
                print("Failed to publish relative rotation value!")
        elif command == 2:
            absolute_rotation = input("Enter the absolute rotation value: ")
            try:
                tpub_absolute_rotation.publish(Float32MultiArray(data=[absolute_rotation]))
            except rospy.ROSException:
                print("Failed to publish absolute rotation value!")
        elif command == 3:
            try:
                tpub_calibration_done.publish(Bool(data=True))
            except rospy.ROSException:
                print("Failed to publish calibration command!")
        else:
            print("Invalid command! Please enter a valid command number.")