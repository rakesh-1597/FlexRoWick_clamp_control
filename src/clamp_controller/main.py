#!/usr/bin/env python3
from controller_interface.clamp_command_interface import ClampCmdInterface

# This script provides a command-line interface for controlling the FlexRoWick Clamp using the ClampCmdInterface class.
# Function to handle user input and call the appropriate methods on the ClampCmdInterface object
# This function will run in a loop, allowing the user to enter commands repeatedly
def userInputLoop(interfaceObj:ClampCmdInterface):
    print("Welcome to the FlexRoWick Clamp Controller terminal!")
    while True:
        try:
            print("Enter the command number (1-6):")
            print("1: Perform relative rotation")
            print("2: Perform absolute rotation")
            print("3: Perform calibration")
            print("4: Perform clamp initialisation")
            print("5: Stop rotation")
            print("6: Exit")
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
        elif command == 4:
            try:
                flag = int(input("Perform clamp initialisation Flag: 1(True) or 0(False)"))
            except ValueError as e:
                print(f"Error! Entered value is not of type int {e}")
            interfaceObj.initialiseClamp(bool(flag))
        elif command == 5:
            try:
                flag = int(input("Stop rotation Flag: 1(Stop) or 0(Continue)"))
            except ValueError as e:
                print(f"Error! Entered value is not of type int {e}")
            interfaceObj.stopRotation(bool(flag))
        elif command == 6:
            print("Exiting...")
            break
        else:
            print("Invalid command! Please enter a valid command number.")

if __name__ == "__main__":
    clampCmdIf = ClampCmdInterface()
    userInputLoop(clampCmdIf)


    