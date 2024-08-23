# match_web_interface
A webinterface to monitor the MUR620 platforms used at Match

## Description

### dataserver Dictionary
Contains `dataserver.py` and `start_dataserver.sh`. The dataserver listens to the battery status of the Raspberry Pi via ROS Topics and provides a REST API to send the metrics to the frontend. The script also checks if the Raspberry Pi is reachable to prevent erros at boot. It needs to be added to systemd to start on boot on the MUR platforms (see below). If it returns data the webinterface will consider the MUR as Online.

### proxy Dictionary
Contains `server.js` which acts as backend for the webinterface. It will read the data from the MIR API combined with the dataserver.py on the MURs and broadcast the data so the `getdata.js` script in the frontend can access the informations. The script needs to be started with NodeJS and an autostart must be added.

### js Dictionary
Contains `getdata.js` and `settings.js`. The first JavaScript is the frontend of the webinterface and reads the data from the Proxy Server on the same machine. The values will be automatically changed and updated for the HTML file. Bootstrap5 needs to be installed in this dictionary (5.3.3 was used, see below for tutorial). The setting script is currently a placeholder for futher extensions and should not be used!

## Setup

### 1. Install the webserver and bootstrap
Install apache2 on the server `sudo apt install apache2`

Copy the `js` and `image` folder with the `index.html`, `settings.html` and `styles.css` files into `/var/www/html/` directory

Change in the `getdata.js` file the variable `response` to the local ip of the machine

Download the compiled files of [bootstrap5.3](https://getbootstrap.com/docs/5.3/getting-started/download/)

Add the bootstrap directory to your `/var/www/html/` directory (booth bootstrap and `getdata.js` will be in the `js` folder)

Check the website by typing in `localhost` in your browser, for problems check with F12

### 2. Setup the server.js as backend
WIP

### 3. Install the dataserver with autostart on MUR
Assuming ROS Noetic is installed and the Raspberry Pi is working

Copy the dataserver directory into `/home/rosmatch/dataserver`

Configure `RASPI_ROS_MASTER_IP` and `LOCAL_IP` in the `dataserver.py` file

Enable network capabilities for python `sudo setcap cap_net_raw+ep /usr/bin/python3.8`

Install the libaries with pip `pip install flask asyncio`

Test the dataserver by running `python3 dataserver.py` and check the webinterface

Create the autostart systemd service:

`sudo nano /etc/systemd/system/dataserver.service`

```                        
[Unit]
Description=Dataserver Service
After=network.target

[Service]
ExecStart=/home/rosmatch/dataserver/start_dataserver.sh
ExecStop=/bin/kill -s SIGTERM $MAINPID
Restart=always
User=rosmatch
Environment=PYTHONUNBUFFERED=1
StandardOutput=syslog
StandardError=syslog
WorkingDirectory=/home/rosmatch/dataserver
KillMode=control-group
TimeoutStopSec=10
KillSignal=SIGKILL

[Install]
WantedBy=multi-user.target
```

`sudo systemctl daemon-reload`

`sudo systemctl enable dataserver.service`

Reboot and test with `sudo systemctl status dataserver.service`
