# Ubuntu
Is better for ROS compared to RasPiOS.

## Deploy
Install and run `Raspberry Pi Imager`.

* Device: Raspberry Pi Zero 2 W
* OS: Other general-purpose OS / Ubuntu / Ubuntu Server 24 (64-bit)
* Storage: your micro-SD card
* etc...

Insert the card into raspi, start it and get terminal (either real or ssh).

## Setup
### Recommended to add swap first
The RAM is limited, so it is very possible to exceed the limit. 
Below 1GB is added, but you can select more.
```
sudo dd if=/dev/zero of=/swapfile bs=1M count=1024
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Optionally install Avahi
This will help you to find the robot by `<hostname>.local`
```
sudo apt install avahi-daemon
```

### Optionally install Raspi Config:
```
sudo apt-get install raspi-config
sudo raspi-config
```

#### Enable Serial Port
* Interface Options
  * Serial Port
    * Login Shell: no
    * Serial Port: yes

#### Disable bluetooth
This is needed to enable HW UART (ttyAMA0) instead of SW UART (ttyS0).
```
sudo nano /boot/firmware/config.txt 

# Disable BT
dtoverlay=disable-bt
```

#### Enable legacy camera
There was an error `FATAL Serializer control_serializer.cpp:605 A list of V4L2 controls requires a ControlInfoMap` when trying to use `camera_ros camera_node`. So it is easier to fall back to legacy appraoch:
```
# comment out default settings
#dtoverlay=vc4-kms-v3d
#disable_fw_kms_setup=1

# libcamera is disabled
#camera_auto_detect=1
#display_auto_detect=1

# legacy camera driver is enabled
camera_auto_detect=0
start_x=1
gpu_mem=64

[pi02]
# set cma to 64 MB too 
dtoverlay=vc4-kms-v3d,cma-64
```

### Clean packages
```
sudo apt remove cloud-* multipath-tools packagekit polkitd rsyslog snapd unattended-upgrades
```

### Optionally disable Ubuntu features
```
sudo nano /etc/default/apport

# set
enabled=0
```

### Fix Errors
Fix the "System is booting up. Unprivileged users are not permitted to log in yet.":
```
sudo nano /etc/pam.d/login
# comment out:  auth  requisite  pam_nologin.so
```

Fix failure waiting for wired network:
```
sudo mkdir -p /etc/systemd/system/systemd-networkd-wait-online.service.d/
sudo nano /etc/systemd/system/systemd-networkd-wait-online.service.d/override.conf

# save the file with following contents
[Service]
ExecStart=
ExecStart=/lib/systemd/systemd-networkd-wait-online --interface=wlan0
```
