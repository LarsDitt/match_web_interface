# match_webinterface
A webinterface to monitor the MUR620 platforms used at Match

## Description

### dataserver Dictionary
Contains `dataserver.py` and `start_dataserver.sh`. This script is the main script of the dataserver. It listens to the battery status of the Raspberry Pi and provides a REST API to send the metrics to the frontend. The script also checks if the Raspberry Pi is reachable It needs to be added to systemd to start on boot (see below). It runs on the MUR platforms. If it returns data the webinterface will consider the MUR as Online.

### proxy Dictionary
Contains `server.js` which acts as backend for the webinterface. It will read the data from the MIR API combined with the dataserver.py on the MURs and broadcast the data so the `getdata.js` script in the frontend can access the informations. The script needs to be started with NodeJS and an autostart must be added.

### js Dictionary
Contains `getdata.js` and `settings.js`. The first JavaScript is the frontend of the webinterface and reads the data from the Proxy Server on the same machine. The values will be automatically changed and updated for the HTML file. Bootstrap5 needs to be installed in this dictionary (5.3.3 was used, see below for tutorial). The setting script is currently a placeholder for futher extensions.

## Setup

### Install the webserver and bootstrap
WIP

### Setup the server.js as backend
WIP

### Install the dataserver with autostart on MUR
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