# Ubuntu
can be used instead of RasPiOS.

## Setup
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

#### Important Note
One time raspi-config made OS non-bootable, so the following settings are maybe better to change manually:
`/boot/firmware/config.txt` (`/boot/config.txt`).

The menu above probably does `enable_uart=1`, but somehow it did not work without raspi-config.

#### Disable bluetooth
This is needed to enable HW UART (ttyAMA0) instead of SW UART (ttyS0).
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

Fix failure waiting for wired network:
```
sudo mkdir -p /etc/systemd/system/systemd-networkd-wait-online.service.d/
sudo nano /etc/systemd/system/systemd-networkd-wait-online.service.d/override.conf

# save the file with following contents
[Service]
ExecStart=
ExecStart=/lib/systemd/systemd-networkd-wait-online --interface=wlan0
```
