# dev_sts_pi
This is the repository for the University of York's Computer Science Open Day demonstration. This repository is intended to work with Ubuntu 20.04, ROS2 - Foxy Fitzgerald and the STS-Pi Roving Robot


## Requirements
  - Raspberry Pi 4
  - Ubuntu 20.04 Focal Fossa (Currently tested using Ubuntu MATE 20.04 on all robot variations)
  - Python 3.8.5
  - openssh-client openssh-server net-tools python3-pip  python3-smbus git 
  * pip3 install explorerhat opencv-python opencv-contrib-python
  - ros2 (Foxy Fitzgerald) ~ https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
  - sudo apt install python3-colcon-common-extensions (Must be installed after ROS2)


## Installation
NOTE: This is an extremely comphrensive set of installation steps such that this can be recreated for years to come. For any technical difficulties, please contact arn519@york.ac.uk or alan.millard@york.ac.uk
  * Flash Ubuntu (or Ubuntu MATE) 20.04 onto an SD card
  * Insert SD card into the Pi 4 and connect a monitor, keyboard and mouse and boot the Pi
  * Follow through installation of OS
  * sudo apt install -y openssh-client openssh-server net-tools python3-pip  python3-smbus git 
  * pip3 install explorerhat opencv-python opencv-contrib-python
  * Install ROS2 Foxy
  * Run sudo apt install -y python3-colcon-common-extensions
  * Clone the Repository: git clone https://github.com/AidenNeale/dev_sts_pi.git
  * Move into the repository: cd dev_sts_pi/
  * colcon build --symlink-install
  * Set environment variable on robot to determine role of robot, e.g. driver_1, follower_1, etc. 
  `echo "export ROBOT_ID=driver_1" >> ~/.bashrc`
  * Look at Usage steps to use robot

## Usage
- Assuming correct installation and setup, create three terminals and ssh into two robots.
- On each robot
  - Navigate to the root directory of the project (/dev_sts_pi)
  - Run: bash setup_perms.sh and enter the password when prompted
  - Run: source install/setup.bash
  - Run: ros2 launch py_sts_pi sts.launch.py
- On the third terminal:
  - Navigate to the root directory of the project (/dev_sts_pi)
  - Run: source install/setup.bash
  - Run: ros2 launch py_sts_pi computer.launch.py

## To Do


## Troubleshooting

