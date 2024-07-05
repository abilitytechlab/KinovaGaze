# KinovaGaze

KinovaGaze is a system for controlling a Kinova Jaco robotic arm using gaze.

# Setup

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

## Setup swap file
The Raspberry Pi 3 only has 1GB of RAM, which can cause issues during installation. As such it is recommended to setup a swap file. The commands below will install a 3GiB swap file, so the total memory available is 4GiB.

```
sudo dd if=/dev/zero of=/swapfile bs=1024 count=3145728
sudo mkswap /swapfile
sudo chmod 600 /swapfile
sudo swapon /swapfile
echo -e "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab
```

## Install ROS and setup environment
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

## Setup ROS environment
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

## Install Kinova-ROS Noetic
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

Run catkin_make:
