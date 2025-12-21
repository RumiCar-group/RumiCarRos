# Ubuntu
can be used instead of RasPiOS.

## Setup
### Recommended to add swap first
```
sudo dd if=/dev/zero of=/swapfile bs=1M count=1024
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
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

### Clean packages
```
sudo apt remove cloud-* multipath-tools packagekit polkitd rsyslog snapd unattended-upgrades
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
