# FlexRoWick clamp control Workspace

This repository is part of **FlexRoWick** project for controlling the clamp, which integrates the **FreeOpcUa** library and custom ROS packages like `clamp_control` to provide functionality for controlling devices via OPC UA in a simulated or real-world environment.

PLC IP: 172.31.1.160

## Prerequisites

Ensure that the following dependencies are installed on your system before proceeding:

### General Tools
- **Git**
- **CMake** (version 3.0 or later)
- **Python** (version 3.8.10 or later)

### ROS Setup
- ROS Noetic (required)

Install ROS Noetic:
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Source the ROS environment:
```bash
source /opt/ros/noetic/setup.bash
```

## Clone the Repository

Create a new ROS workspace and clone this repository:
```bash
mkdir -p ~/FlexRoWick_clamp_ws/src
cd ~/FlexRoWick_clamp_ws/src

git clone https://github.com/rakesh-1597/FlexRoWick_clamp_control.git
```
## Build the Workspace

### Step 1: Clean the Workspace
(Optional, but recommended if you have previous builds)
```bash
cd ~/FlexRoWick_clamp_ws
rm -rf build devel
```

### Step 2: Build with Catkin
Run the following command to build the workspace:
```bash
cd ~/FlexRoWick_clamp_ws
catkin_make
```

### Step 3: Source the Workspace
After a successful build, source the workspace to update your environment:
```bash
source ~/FlexRoWick_clamp_ws/devel/setup.bash
```

Add this to your `~/.bashrc` file to source automatically:
```bash
echo "source ~/FlexRoWick_clamp_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## Pre-Conditions
### Ensure that the PLC and the clamp is powered ON, PLC is in 'RUN' state

## Running the ROS Nodes
### Step 1: Run roscore in a terminal
From root directory of the machine, source the setup.bash script
```bash
source /opt/ros/noetic/setup.bash
```

Run roscore
```bash
roscore
```

### Step 2: Running `plc_client_node`
Move to root directory of the project folder, source the setup.bash script of the project present under 'devel' folder
```bash
source devel/setup.bash
```

from root directory of the project, move to 'src/clamp_controller/plc_client_node/' and run plcClientNode.py python script
```bash
$ cd src/clamp_controller/plc_client_node/
$ python3 plcClientNode.py
```
This will run the plc_client_node ROS node, that will be waiting for the commands to be executed which are to be sent using 'clamp_command_interface' ROS node.

### Step 3: Running 'clamp_command_interface' ROS node to execute clamp init and actuation
#### Execute 'main.py' script
'main.py' script present in 'src/clamp_cointroller' sub-folder uses methods of 'ClampCmdInterface' class (check src/controller_interface/clamp_command_interface.py).
These methods are used to execute operations as mentioned in the points below.
```bash
$ cd src/clamp_controller/
$ python3 main.py
```
This will output the following in the terminal
```bash
$ Welcome to the FlexRoWick Clamp Controller terminal!
Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration 
 4) Perform clamp initialisation 5) Stop rotation
Enter the command number:
```
#### Activate Pre-charging and In-Feed
Enter '4' in the command terminal and give 1
```bash
$ Welcome to the FlexRoWick Clamp Controller terminal!
Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration 
 4) Perform clamp initialisation 5) Stop rotation
Enter the command number: 4

$ Perform clamp initialisation Flag: 1(True) or 0(False)1
```

#### Calibrate the clamp
Manually move the clamp to align it to top position as shown in the picture below, using the TIA portal software or the HMI panel. Once the clamp is properly aligned, in the command menu enter 3
```bash
Welcome to the FlexRoWick Clamp Controller terminal!
Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration 
 4) Perform clamp initialisation 5) Stop rotation
Enter the command number: 3
```
Once the calibration command is executed, the operations below can be performed
**Note**: Without performing calibration, one cannot execute any of the rotation operations mentioned below.
#### Execute relative rotation
To perform relative rotation, enter '1' in the command menu
```bash
Welcome to the FlexRoWick Clamp Controller terminal!
Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration 
 4) Perform clamp initialisation 5) Stop rotation
Enter the command number: 1
```
Provide the rotation angle relative to the current position that you want the clamp to move
```bash
Welcome to the FlexRoWick Clamp Controller terminal!
Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration 
 4) Perform clamp initialisation 5) Stop rotation
Enter the command number: 1
Enter the relative rotation value: (any positive or negative float value)
```
Observe the clamp rotation, and note down the debug prints in the terminal which runs 'plcClienNode'. Observe the changes in the current angle of the clamp that gets printed and sent as a ROS topic every 500ms (configurable in the plcClientNode class)


#### Execute absolute rotation
To perform absolute rotation, enter '1' in the command menu
```bash
Welcome to the FlexRoWick Clamp Controller terminal!
Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration 
 4) Perform clamp initialisation 5) Stop rotation
Enter the command number: 2
```
Provide the absolute angle position that you want the clamp to move to
```bash
Welcome to the FlexRoWick Clamp Controller terminal!
Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration 
 4) Perform clamp initialisation 5) Stop rotation
Enter the command number: 2
Enter the absolute rotation value: (any positive or negative float value)
```
Observe the clamp rotation, and note down the debug prints in the terminal which runs 'plcClienNode'. Observe the changes in the current angle of the clamp that gets printed and sent as a ROS topic every 500ms (configurable in the plcClientNode class)

#### Stop rotation midway
While the clamp is executing the rotations, you could also stop the rotation midway using the command number '5'
```bash
Welcome to the FlexRoWick Clamp Controller terminal!
Command Menu: 1) Perform relative rotation 2) Perform absolute rotation 3) Perform calibration 
 4) Perform clamp initialisation 5) Stop rotation
Enter the command number: 5
```
Observe and verify stoppage of rotation, as well as the current angle in the terminal which runs 'plcClienNode'.

## Troubleshooting

### Issue: `Segmentation fault (core dumped)`
Ensure that:
- All dependencies (e.g., FreeOpcUa, spdlog) are installed and linked correctly.
- The OPC UA server is running and accessible.

Run the application in `gdb` for detailed debugging:
```bash
gdb ~/FlexRoWick_clamp_ws/devel/lib/clamp_control/flexrowick_clamp_control
```

### Issue: Library Not Found (`libopcuacore.so`)
Ensure the `LD_LIBRARY_PATH` includes the directory where the library is installed:
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```
Add it to `~/.bashrc` for persistence:
```bash
echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

## Contributing

Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a feature branch.
3. Submit a pull request with a detailed explanation of your changes.

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For questions or support, please contact:
- **Rakesh Bhat Karkala**
- Email: bhatrakesh930@gmail.com
