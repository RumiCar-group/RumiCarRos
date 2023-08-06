# RumiCar system files

## Autostart
Put `systemd/rcar.service` to `~/.config/systemd/user/rcar.service` and enable it:

```
systemctl --user enable rcar.service
systemctl --user start rcar.service
```
