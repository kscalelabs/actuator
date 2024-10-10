# CH340 CAN module

This is the C++ code with a demo for interfacing with the Robstride CAN module.

Notes:

- MacOS doesn't support a baudrate of 921600 required by the Robstride CAN module out of the box. Need to install [this driver](https://github.com/WCHSoftGroup/ch34xser_macos).
- Jetson default image has issues out of the box. Need to install [this driver](https://github.com/WCHSoftGroup/ch341ser_linux).
- Make sure to set the TTY device baudrate to 921600:
- If the device doesn't appear in `/dev`, check `dmesg` to make sure it isn't being masked by another service, like `brltty`
- Make sure that the user is in the `dialout` group and that this group exists:
  - To add to the dialout group: `sudo adduser ${USER} dialout`
  - To make sure the group exists: `newgrp dialout`

To set the baudrate on Jetson:

```bash
sudo stty -F /dev/ttyCH341USB0 921600
sudo stty -F /dev/ttyUSB0 921600
```

To set the baudrate on MacOS:

```bash
stty -f /dev/tty.usbserial-110 921600
```
