# INSTALL ENVIRONMENT

## 0. Prerequisite
- Odroid N2+
- Ubuntu: 20.04 LTS (focal base)

## 1. Install ROS Noetic
### 1.1 Setup your sources.list

Setup your computer to accept software from packages.ros.org.

```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 1.2 Set up your keys
```
    sudo apt install curl -y # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
### 1.3 Installation

First, make sure your Debian package index is up-to-date:

```
    sudo apt update
```
**ROS-Base**: (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools.

```
    sudo apt install ros-noetic-ros-base
    sudo apt install ros-noetic-laser-geometry
    sudo apt install ros-noetic-diagnostic-updater
```
### 1.4 Environment setup

You must source this script in every **bash** terminal you use ROS in.
```
    source /opt/ros/noetic/setup.bash
```
It can be convenient to automatically source this script every time a new shell is launched. These commands will do that for you.

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Build lidar software and install environment
### 2.1 Copy the source code to Odroid
- Folder Structure
    ```
	.
	├── build
	├── devel
	├── install
	├── README.md
	└── src
	    ├── CMakeLists.txt
	    └── lidar_scan
    ```
- Remove build folder before make
    ```
    rm -rf build
    ```
- Build ROS packages
    ```
    catkin_make
    ```
## 4. Run lidar software
- Run the script
    ```
    ./lidar_start.sh
    ```

## 5. Set `systemd` for rrt service
- Create `lidar.service`
    ```
    sudo nano /etc/systemd/system/lidar.service
    ```
- Copy text below to `lidar.service`
    ```
    [Unit]
    Description=Lidar software

    Wants=network.target
    After=syslog.target network-online.target

    [Service]
    Type=simple
    ExecStart=/usr/src/lidar/lidar_start.sh
    Restart=always

    [Install]
    WantedBy=multi-user.target
    ```

- Reload service
    ```
    sudo systemctl daemon-reload
    ```
- Start `lidar.service`
    ```
    sudo systemctl start lidar
    ```
- Enable `lidar.service` for auto start when turn on
    ```
    sudo systemctl enable lidar
    ```

## 7. Build Debian package for update sortware
### 7.1 1. Create the working directory
Create a temporary working directory to make your package in. Follow the same naming convention as described below. For example:
	```
    <name>_<version>-<revision>_<architecture>
    ```

That is:
- `<name>` – the name of your application.
- `<version>` – the version number of your application.
- `<revision>` – the version number of the current deb package.
- `<architecture>`– the hardware architecture your program will be run on.

    ```
    mkdir -p lidar_1.0-1_arm64
    ```
### 7.2. Create the manage file
#### 7.2.1 Control file
```
mkdir -p lidar_1.0-1_arm64/DEBIAN
touch lidar_1.0-1_arm64/DEBIAN/control
```
Open the control file with your text editor of choice. The control file is just a list of data fields. For binary packages there is a minimum set of mandatory ones:

- `Package` – the name of your program; 
- `Version` – the version of your program; 
- `Architecture` – the target architecture; 
- `Maintainer` – the name and the email address of the person/group in charge of the package maintenance; 
- `Description` – a brief description of the program.

#### 7.2.2 Preinstall file
```
touch lidar_1.0-1_arm64/DEBIAN/preinst
sudo chmod +x lidar_1.0-1_arm64/DEBIAN/preinst
```
This script is executed before the package it belongs to is unpacked from its Debian archive (".deb") file. The 'preinst' scripts stop services for packages which are being upgraded until their installation or upgrade is completed (following the successful execution of the 'postinst' script).

For example:
```
	#! /bin/bash
	echo "stop lidar service"
	sudo systemctl stop lidar

	echo "Looking for old versions of rrt ..."

	if [ -d "/usr/src/lidar_v0.4" ];then
	    sudo rm -rf /usr/src/lidar_v0.4
	    echo "Removed old lidar from /usr/src ..."
	fi
```
#### 7.2.3 Postinstall file
```
touch lidar_1.0-1_arm64/DEBIAN/postinst
sudo chmod +x lidar_1.0-1_arm64/DEBIAN/postinst
```
This script typically completes any required configuration of the package once rrt has been unpacked from its Debian archive (".deb") file. Often, 'postinst' scripts ask users for input, and/or warn them that if they accept default values, they should remember to go back and re-configure that package as needed. The 'postinst' scripts then execute any commands necessary to start or restart a service once a new package has been installed or upgraded.

For example:
```
	#! /bin/bash

	echo "over wrire lidar.service"
	sudo cp /usr/src/lidar/lidar.service /etc/systemd/system/

	echo "daemon-reload lidar service"
	sudo systemctl daemon-reload

	echo "start lidar service"
	sudo systemctl start lidar
```

### 7.3. Copy new software version to work directory\
```
mkdir -p lidar_new_version lidar_1.0-1_arm64/usr/src
cp -r lidar_new_version lidar_1.0-1_arm64/usr/src/rrt
```

### 7.4. Build the deb package
```
dpkg-deb --build --root-owner-group lidar_1.0-1_arm64
```
The `--root-owner-group` flag makes all deb package content owned by the root user, which is the standard way to go. Without such flag, all files and folders would be owned by your user, which might not exist in the system the deb package would be installed to.
The command above will generate a nice .deb file alongside the working directory or print an error if something is wrong or missing inside the package. If the operation is successful you have a deb package ready for distribution.

### 7.5. Test your deb package
```
dpkg -i lidar_1.0-1_arm64.deb
```
After installing, you can check the software is updated
```
dpkg -s lidar
```
