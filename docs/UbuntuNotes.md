# Ubuntu
can be used instead of RasPiOS.

## Setup
### Optionally install Raspi Config:
```
sudo apt-get install raspi-config
```

Though it might be better to directly edit `/boot/firmware/config.txt` (`/boot/config.txt`).

#### Disable bluetooth
```
# Disable BT
dtoverlay=disable-bt
```

### Clean packages
```
sudo apt remove bluez pi-bluetooth  # BT related
sudo apt remove cloud-* multipath-tools packagekit polkitd rsyslog snapd unattended-upgrades
```

### Fix Errors
Fix the "System is booting up. Unprivileged users are not permitted to log in yet.":
```
sudo nano /etc/pam.d/login
# comment out:  auth  requisite  pam_nologin.so
```
