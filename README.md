# match_webinterface
A webinterface to monitor the MUR620 platforms used at Match

## Install the dataserver with autostart on MUR
Assuming ROS Noetic is installed and the Raspberry Pi is working

Copy the dataserver directory into `/home/rosmatch/dataserver`

Configure `RASPI_ROS_MASTER_IP` and `LOCAL_IP` in the `dataserver.py` file

Enable network capabilities for python `sudo setcap cap_net_raw+ep /usr/bin/python3.8`

Install the libaries with pip `pip install flask asyncio`

Create the autostart systemd service:

`sudo nano /etc/systemd/system/dataserver.service`

```
[Unit]
Description=Dataserver Service
After=network.target

[Service]
ExecStart=/home/rosmatch/dataserver/start_dataserver.sh
Restart=always
User=rosmatch
Environment=PYTHONUNBUFFERED=1
StandardOutput=syslog
StandardError=syslog
WorkingDirectory=/home/rosmatch/dataserver

[Install]
WantedBy=multi-user.target
```

`sudo systemctl daemon-reload`

`sudo systemctl enable dataserver.service`

Reboot and test with `sudo systemctl status dataserver.service`
