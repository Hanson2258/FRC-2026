# Summary
Goal: have the RealSense camera Raspberry Pi automatically run the post-detection code when it boots. There should be no reason to attach a monitor or ssh into the Pi once this procedure has been followed (alliance colour will be set automatically within the python script).

To accomplish this, we set up a service (depth-cam.service) called by systemd.

# Method
Either SSH into the Pi, or run the following commands from terminal (with keyboard and screen attached to the Pi)

To SSH into the Pi, open a terminal window on a computer connected to the same local network as the Pi (or use an ethernet cable to attach your computer directly to the Pi). Open terminal on your computer, then type the following:
```
ssh eas@RealSense-Pi4B-V2.local
```
It should return with a password prompt. Enter:
```
221
```

## Create service file
Create the following file:
`/lib/systemd/system/depth-cam.service`

File contents should be:
```
[Unit]
Description=Code to run depth cam
After=multi-user.target
[Service]
 Type=idle
Restart=always
User=root
WorkingDirectory=/home/eas/code
ExecStart=/home/eas/venv3/bin/python post-detect.py
[Install]
WantedBy=multi-user.target
```

### Explanation
Most of the action happens in the last two lines of the [Service] section
1. Working Directory is set explicitly where python script resides
2. The virtual environment (venv3) is specified so that the venv Python binary is explicitly used, avoiding the need to use a 'source' call to activate venv

## Set File Permissions
Type the following terminal command:
```
sudo chmod 644  /lib/systemd/system/depth-cam.service
```

## Configure `systemd`
Type the following terminal commands:
```
sudo systemctl daemon-reload
sudo systemctl enable depth-cam.service 
sudo systemctl start depth-cam.service 
```

## Finally
- Reboot the Pi (with the RealSense camera attached) and check that the attached RoboRIO sees the NetworkTable "post-detection" appear.
- Note the post-detection python script must always have the name specified in the service file above (regardless of version) and, of course, be located in home/eas/code
- to check and see if the service is running use this command (shows all services and their status):
```
sudo systemctl list-units --type=service
```
- To view the output (log) of the service:
```
journalctl -e -u depth-cam.service
```
- To stop the service:
```
sudo systemctl stop depth-cam.service
```
