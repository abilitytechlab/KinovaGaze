# KinovaGaze

KinovaGaze is a system for controlling a Kinova Jaco robotic arm using gaze.

# Using
Connect the Raspberry Pi to any computer over Ethernet, using a USB to Ethernet adapter if needed. Ensure the robot arm and camera are connected to the Pi over USB, then connect the Raspberry Pi to power. Wait about 2 minutes for the system to start, the hand on the arm will open when the system is ready to use.

Use a web browser on a PC to access the UI. Tested to work on Chromium browsers (Edge, Chrome). Camera feed broken in the latest version of Firefox at the time of writing. Not tested on Webkit (Safari). Not compatible with Internet Explorer. Not tested on touchscreen devices.

Navigate to http://192.168.137.1:5000 to access the default UI (start-stop), other UI pages are listed below.

To use the UI, configure your gaze-tracker software such that the mouse cursor always follows your gaze. Buttons are activated by hovering over them, clicking is not needed.

All regular pages:
- http://192.168.137.1:5000/start-stop
- http://192.168.137.1:5000/hold
- http://192.168.137.1:5000/whack-a-button

All pages with recorder enabled for research purposes:
- http://192.168.137.1:5000/research_start-stop
- http://192.168.137.1:5000/research_hold
- http://192.168.137.1:5000/research_whack-a-button

# Technical overview
A KinovaGaze system consists of an external computer like a Raspberry Pi connected to a Kinova Jaco robot arm over usb as well as a USB camera like a webcam. The system is connected to a gaze-controlled computer over Ethernet, where various web-interfaces are available. See the diagram below.

![System hardware diagram](<System diagram simplified.png>)

Under the hood, the Raspberry Pi runs various services. These services are automatically ran on boot using Systemd, which will also restart them if they exit. A full diagram of the system with the various software and hardware components is included below.

![System software and hardware diagram](<System diagram.png>)

## ROS-Noetic: https://wiki.ros.org/noetic

ROS-Noetic is used as the base for the robot arm control of KinovaGaze. Three systems are ran under ROS: The KinovaGaze ROS node, the Kinova-ROS stack, and rosbridge.

### KinovaGaze ROS Node: `/`. 

The root of this repro is a ROS node. The files which are part of this node are `stripts/`, `CMakeLists.txt` and `package.xml`. Other files are also stored in the node for convenience, but are not part of its function.

The node is responsible for receiving messages from the Web UI via rosbridge and sending commands to the robot arm via the Kinova ROS stack. 

### Kinova-ROS: https://github.com/Kinovarobotics/kinova-ros

The Kinova-ROS stack is installed alongside the KinovaGaze node to communicate with the robot arm. It is accessed via the KinovaGaze node.

### rosbridge_server: https://wiki.ros.org/rosbridge_suite

Rosbridge is the communication layer which translates between ROS messages for the ROS node and WebSocket JSON messages for the GUI.

## Web Interface: `/gui/`

The GUI allows the user to use the system via a web interface.

### GUI Server: `/gui/gui_server.py`

The GUI server is responsible for hosting the GUI webpages, the camera feed and all dependencies. The pages are stored under `/gui/templates/`. Dependencies are stored under `/gui/static/`.

### Web Interface Scripts: `/gui/static/scripts/`

Part of the web interface scripts are scripts containing the main GUI logic. They are written using p5.js (https://p5js.org/), a library for writing custom graphical applications in JavaScript.  `gui_start-stop.js` and `gui_hold.js` contain the main interfaces, and `whack-a-button.js` contains a simple game. It is recommended to study the code to Whack-A-Button before editing the other interfaces, as it is simpler and more thoroughly documented.

Alongside the GUI stripts there are some helper scripts which contain classes used by the GUI. These should be imported into the HTML before importing the GUI script. `gazecontrol.js` contains classes for the gaze-controlled interface system. `roscomm.js` contains a class for communicating with the ROS node. Finally `datarecorder.js` contains an optional class for recording the interface and keeping a log, it is meant for research and only included in the research variations of the webpages.

## Systemd Services: `/services/`

The `.service` files contain the Systemd rules for running the various parts of KinovaGaze, this being Kinova-ROS, the ROS node, rosbridge server and the GUI server. They are copied to `/etc/systemd/system` during install. The DHCP server is also ran via Systemd, but this is already set up when you install ISC DHCP.

### ISC DHCP: https://www.isc.org/dhcp/

ISC DHCP is a program which assignes IP addresses on a local network. This is used to allow any computer connected to the Pi's main Ethernet port to access the interfaces without further setup, but prevents this port from being used to connect to a router.

# Setup from scratch

## Requirements
To setup KinovaGaze from scratch, you'll need:
- A Kinova Jaco robotic arm with power adapter and USB A to B cable. [^1]

[^1]: Specifically a _Jaco v2 6DOF serice with a 3 finger gripper_, although other models may work with slight modification to KinovaGaze.

- A webcam or other camera compatible with OpenCV

- A Raspberry Pi 3b with a >=16GB MicroSD card and an appropriate power adapter (Or a PC with equal or better speficications, although the installation only covers using a Raspberry Pi). [^2]

[^2]: Specifically, the system needs to be able to run the Kinova-ROS stack, so PCs are also support. This requires running Ubuntu 20.04 LTS and the use of pre-compiled libraries which are provided in x86 and x64 variants as well as some specifically for the Raspberry Pi 3 (Note: RPi libraries only support 32-bit operating systems). The Raspberry Pi 4 partly works but has issues with controlling the arm, and earlier Pi models will likely have major performance problems if supported at all. Newer Pi models have not been tested but are likely incompatible. At 1GB of RAM is recommended, although having more will make installation smoother.

- An ethernet cable with (optional) ethernet to USB adapter.

- A means of using Raspberry Pi Imager to install an OS on the MicroSD card

- (Recommended) An external computer to SSH into the Pi for setup

- For initial setup: A display, HDMI cable, keyboard, and a means to connect the Pi to the internet without using its ethernet port. [^3]

[^3]: This could be a WiFi access point with only password protection, a USB to ethernet adapter or using (USB) tethering via a phone. The built-in ethernet port will be running a DHCP server which prevents connecting it to an existing network.

To use the system, you'll need:

- A computer which can connect to the Pi over Ethernet

- A gaze-tracker with support for having the mouse cursor follow the gaze.

## Installation

### Install OS

Install Ubuntu Server 20.04.5 LTS 32-bit onto the Raspberry Pi using Raspberry Pi Imager. Use hostname _kinovagaze_ and username _kinovagaze_ with any password. If you'll use WiFi on the Pi, configure the wireless LAN now.

### Setup network
Initially networking on the Pi will likely not work. After the Pi is booted, connect a display and keyboard to configure the ethernet port and connect to the internet.

Before we do anything, disable Ubuntu's unnatended upgrades. This will only get in the way of working, and these instructions will tell you when you can upgrade everything manually. KinovaGaze is not intended to be connected to the internet during normal use, but if you do keep it connected then re-enable automatic updates after setup is completed.

```sudo systemctl disable --now unattended-upgrades```

Now use `sudo nano /etc/netplan/50-cloud-init.yaml` to create or modify `/etc/netplan/50-cloud-init.yaml`. This will setup the ethernet port on ip `192.168.137.1` and set up WiFi (the `wifis:` portion should already be configured if you setup the wireless LAN via Raspberry Pi Imager.) 

Afterwards `/etc/netplan/50-cloud-init.yaml` should look as follows, leaving out the `wifis:` section is you are not using WiFi:

```
network:
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                [WiFi network SSID]:
                    password: [Wifi Network ]
            dhcp4: true
            optional: true
    ethernets:
        eth0:
            dhcp4: false
            addresses:
            - 192.168.137.1/24
```

If you'll be using USB tethering to connect to the internet, modify `/etc/netplan/01-network-manager-all.yaml` as follows:

```
network:
  version: 2
  renderer: networkd
  ethernets:
    usb0:
      dhcp4: yes
      dhcp6: no
```

After editing the files, enable the changes:
```
sudo netplan apply
```

To see if this was successful, check the network status using `ip a`, it should look something like this with the IP `192.168.137.1/24` assigned to `eth0`:
```
[...]
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether [Physical MAC address] brd ff:ff:ff:ff:ff:ff
    inet 192.168.137.1/24 brd 192.168.137.255 scope global eth0
       valid_lft forever preferred_lft forever
    inet6 [IPv6 address]/64 scope link
       valid_lft forever preferred_lft forever
3: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether [Wireless MAC adress] brd ff:ff:ff:ff:ff:ff
    inet [Wireless IP address]/16 brd 10.0.255.255 scope global dynamic wlan0
       valid_lft 38947sec preferred_lft 38947sec
```

Also check if you can access the internet:
```
ping 8.8.8.8
```

Now that you're connected to the internet, immediately update everything. This is a good idea for security, plus otherwise Ubuntu's unattended-upgrade will likely get in the way of setup.
```
sudo apt update
sudo apt upgrade
```

To be able to access the Pi via kinovagaze.local, install avahi-daemon:
```
sudo apt install avahi-daemon
```

To be able to directly connect to a computer via the local ethernet port we'll use isc-dhcp-server.
Install it and then make a backup of the configuration files:
```
sudo apt install isc-dhcp-server
sudo mv /etc/dhcp/dhcpd.conf{,.bak}
sudo mv /etc/default/isc-dhcp-server{,.bak}
```

Now modify `/etc/dhcp/dhcpd.conf` as follows:
```
default-lease-time 600;
max-lease-time 7200;
ddns-update-style none;
authoritative;

subnet 192.168.137.0 netmask 255.255.255.0 {
  interface eth0;
  range 192.168.137.100 192.168.137.200;
  option routers 192.168.137.1;
}
```

And modify `/etc/default/isc-dhcp-server` as follows:
```
INTERFACESv4="eth0"
INTERFACESv6=""
```

Reboot the Pi to finish network setup:
```
sudo reboot
```

You can also check the status of isc-dhcp-server and ensure it says `Active: active (running)`:
```
sudo systemctl status isc-dhcp-server.service
```

If everything's okay, you can now connect to the Pi over SSH on your local machine:
```
ssh kinovagaze@kinovagaze.local
```

### Setup swap file
The Raspberry Pi 3 only has 1GB of RAM, which can cause issues during installation. As such it is recommended to setup a swap file. The commands below will install a 3GiB swap file, so the total memory available is 4GiB.

```
sudo dd if=/dev/zero of=/swapfile bs=1024 count=3145728
sudo mkswap /swapfile
sudo chmod 600 /swapfile
sudo swapon /swapfile
echo -e "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab
```

### Install ROS and setup environment
To install ROS you can follow the instructions over at https://wiki.ros.org/noetic/Installation/Ubuntu, or you can follow the specific instructions below.

Setup sources for ROS:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Install curl and then setup ROS keys:
```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Now insall ROS noetic base:
```
sudo apt install ros-noetic-ros-base
```

Your terminals will need to source ROS to use its commands, this will do this automatically:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Setup ROS install dependencies:
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python-is-python3
```

Setup rosdep:
```
sudo rosdep init
rosdep update
```

### Setup ROS environment
This is the workspace in which we'll be installing most of KinovaGaze. Use the instructions over at https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment or follow the instructions below.

Create the workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

Automatically source the workspace for every terminal:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Kinova-ROS Noetic
This is the ROS stack for controlling the arm. Documentation is over at https://github.com/Kinovarobotics/kinova-ros. The installation instructions below are based on the ones found there, but modified for Kinovagaze.

Clone Kinova-ROS:
```
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/Kinovarobotics/kinova-ros.git
```

Install dependencies:
```
sudo apt install libopenblas-dev ros-noetic-eigen-conversions
```

The following components are needed, otherwise `catkin_make` will fail. Alternatively you can try deleting `~/catkin_ws/src/kinova-ros/kinova_moveit`.
```
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-trac-ik
```

Run catkin_make (if you only have 1GB of RAM and no swap, replace all `catkin_make` commands with `catkin_make -j1`):
```
cd ~/catkin_ws
catkin_make
```

Setup Kinova USB rules:
```
sudo cp ~/catkin_ws/src/kinova-ros/kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/
```

We'll need to install shared libraries for Raspberry Pi 3. First clone the repository containing them, then copy them to the right folder and finally delete the remaining files.
```
mkdir ~/catkin_ws/src/kinova-ros/kinova_driver/lib/arm-linux-gnueabihf
cd ~
git clone https://github.com/Kinovarobotics/kinova_sdk_recompiled
cp "~/kinova_sdk_recompiled/raspberry pi 3/_SO/. ~/catkin_ws/src/kinova-ros/kinova_driver/lib/arm-linux-gnueabihf
rm -rf ~/kinova_sdk_recompiled
```

The new files are named incorrectly, rename them:
```
cd ~/catkin_ws/src/kinova-ros/kinova_driver/lib/arm-linux-gnueabihf
mv Kinova.API.CommLayerUbuntu.so USBCommLayerUbuntu.so
mv Kinova.API.EthCommLayerUbuntu.so EthCommLayerUbuntu.so
mv Kinova.API.USBCommandLayerUbuntu.so USBCommandLayerUbuntu.so
mv Kinova.API.EthCommandLayerUbuntu.so EthCommandLayerUbuntu.so
```

### Install rosbridge_suite
Install the communication systen between ROS and the web interface.
```
sudo apt install ros-noetic-rosbridge-suite
cd ~/catkin_ws
catkin_make
```

### Install KinovaGaze files
Now we can set up the custom files of KinovaGaze.

Clone and install the KinovaGaze files. Replace [url] with the repository where you found this readme file.
```
cd ~/catkin_ws/src
git clone [url]
```

We'll need to set up python with the right modules. Unfortunately I had a lot of issues doing this in a clean, reproducable way, as such I cannot give much help with this. I was unable to set up a Python Virtual Environment, and therefore also unable to test which versions of which modules needed to be installed. Trying to install or update a lot of dependencies causes issues with the pre-installed packages (especially numpy and cmake), so you may have to use apt to uninstall the old system versions of the packages because else you'll likely get vague error messages when trying to install. Directly using the requirements.txt file will likely not work and it might be incomplete or incorrect, but it should serve as a general guide of what needs to be installed. Finally: Installing some of these modules (especially numpy) will take a long time (might be up to an hour!), so keep that in mind.

To get your webcam working, you might need to experiment with changing some code in the method `def video_feed(self)` of the file `~/catkin_ws/src/kinovagaze/gui/gui_server.py`.

### Setup auto start on boot
Finally we'll install all needed programs as services so that they'll automatically start on boot.

```
cp -r ~/catkin_ws/src/kinovagaze/services/. /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable kinova-ros.service
sudo systemctl enable kinovagaze_gui_server.service
sudo systemctl enable kinovagaze_node.service
sudo systemctl enable rosbridge_server.service
```

The service isc-dhcp-server is supposed to start automatically on boot, but sometimes it fails. You can change `/lib/systemd/system/isc-dhcp-server.service` to make it restart after exiting by changing the file to look like this:
```
[Service]
   [...]
Restart=on
RestartSec=10s

[...]
```

# Finishing touches

It is recommended to re-enable automatic updates if the system will be connected over a (non-metered) internet connection during use.

```
sudo systemctl enable --now unattended-upgrades
```

To remove the WiFi connection while the system is installed, edit `/etc/netplan/50-cloud-init.yaml` to remove everything under `wifis:` and then run `sudo netplan apply`

To update KinovaGaze in the future:
```
cd ~/catkin_ws/src/kinovagaze
git pull
```