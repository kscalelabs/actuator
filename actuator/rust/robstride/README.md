# Set up
Install first the CH340 driver from https://github.com/WCHSoftGroup/ch341ser_linux
Also set the baud rate to 921600 with:
```bash
sudo stty -F /dev/ttyCH341USB0 921600
```

Sometimes the permissions are restrictive and tty won't be available, so you may need to run:
```bash
sudo chmod 666 /dev/ttyCH341USB0
```
