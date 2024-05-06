# RumiCar system files

## Autostart
Copy service file and enable it:

```
ssh pi@rcar.local

cd this_package_location
mkdir -p ~/.config/systemd/user/ && cp systemd/rcar.service "$_"

# edit RCAR_LAUNCH if needed

sudo loginctl enable-linger  # so service doesn't require login
systemctl --user enable rcar.service
systemctl --user start rcar.service
```
