# RumiCar system files

## Autostart
### Copy service file

```
ssh pi@rcar.local

cd this_package_location
mkdir -p ~/.config/systemd/user/ && cp systemd/rcar.service "$_"

# edit RCAR_LAUNCH if needed
nano ~/.config/systemd/user/rcar.service
```

### Enable it
```
sudo loginctl enable-linger  # so service doesn't require login
systemctl --user enable rcar.service
systemctl --user start rcar.service
```

### Check logs
```
journalctl --user-unit rcar
```
