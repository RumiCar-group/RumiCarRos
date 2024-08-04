## Server
```
sudo apt install tightvncserver

vncserver :0
```

This will create an X session on :0 virtual desktop. The default port is used: `5900`.

If you skip `:0` parameter, it will create `:1` desktop on port `5901`.
To run applications there, specify default desktop, e.g. `export DISPLAY=:1`.

## Client
```
sudo apt install remmina  # or any other VNC client
```

Server: rcar.local[:5900]
User: pi
Password: <specified in vncserver above>
