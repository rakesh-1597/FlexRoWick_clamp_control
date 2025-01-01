# FlexRoWick clamp control Workspace

This repository is part of **FlexRoWick** project for controlling the clamp, which integrates the **FreeOpcUa** library and custom ROS packages like `clamp_control` to provide functionality for controlling devices via OPC UA in a simulated or real-world environment.

## Prerequisites

Ensure that the following dependencies are installed on your system before proceeding:

### General Tools
- **Git**
- **CMake** (version 3.0 or later)
- **GCC/G++** (version 9 or later)

### ROS Setup
- ROS Noetic (recommended)

Install ROS Noetic:
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Source the ROS environment:
```bash
source /opt/ros/noetic/setup.bash
```

### FreeOpcUa Dependencies
- **spdlog**
- **Boost** libraries

Install dependencies:
```bash
sudo apt install libspdlog-dev libboost-system-dev libboost-thread-dev
```

## Clone the Repository

Create a new ROS workspace and clone this repository:
```bash
mkdir -p ~/FlexRoWick_clamp_ws/src
cd ~/FlexRoWick_clamp_ws/src

git clone https://github.com/rakesh-1597/FlexRoWick_clamp_control.git
```

Clone the **FreeOpcUa** library into the workspace:
```bash
git clone https://github.com/FreeOpcUa/freeopcua.git
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

## Running the ROS Nodes

### Example: Running `clamp_control`
To run the `clamp_control` node, use:
```bash
rosrun clamp_control flexrowick_clamp_control
```

Ensure that the OPC UA server is running and reachable at the specified endpoint configured in your node.

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