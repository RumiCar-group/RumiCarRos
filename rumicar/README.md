# RumiCar system files

## Autostart
Copy service file and enable it:

```
ssh pi@rcar.local

cd this_package_location
cp systemd/rcar.service ~/.config/systemd/user/rcar.service

# edit RCAR_LAUNCH if needed

loginctl enable-linger  # so service doesn't require login
systemctl --user enable rcar.service
systemctl --user start rcar.service
```
