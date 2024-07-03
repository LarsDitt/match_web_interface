# match_webinterface
match_webinterface


# Create service for dataserver on MUR
Copy the dataserver dictionary into `/home/rosmatch/dataserver` on the Mur

Create the autostart systemd service:

`sudo nano /etc/systemd/system/dataserver.service`

```
[Unit]
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
