#!/usr/bin/env python3
from controller_interface.clamp_command_interface import ClampCmdInterface

def userInputLoop(interfaceObj:ClampCmdInterface):
    while True:
        print("Welcome to the FlexRoWick Clamp Controller terminal!")
        print("Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration")
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

if __name__ == "__main__":
    clampCmdIf = ClampCmdInterface()
    userInputLoop(clampCmdIf)


    