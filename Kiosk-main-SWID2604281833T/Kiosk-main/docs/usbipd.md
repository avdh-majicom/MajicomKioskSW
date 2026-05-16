# USB Attach/Detach (WSL)

These are the minimal PowerShell commands to share a USB device with WSL and then detach it.

## Attach (share + attach)
1) List devices and find the BUSID:
   ```
   usbipd list
   ```
2) Share the device (only needed once per reboot):
   ```
   usbipd bind --busid <BUSID>
   ```
3) Attach to WSL (all distros):
   ```
   usbipd attach --wsl --busid <BUSID>
   ```
   If you have multiple distros:
   ```
   usbipd attach --wsl --busid <BUSID> --distribution <DistroName>
   ```

## Detach (detach + unshare)
1) Detach from WSL:
   ```
   usbipd detach --busid <BUSID>
   ```
2) Unshare (optional, if you want to clear it):
   ```
   usbipd unbind --busid <BUSID>
   ```

## WSL side: find the serial device
Once attached, in WSL:
```
ls /dev/ttyACM* /dev/ttyUSB*
```

Common ports are `/dev/ttyUSB0` or `/dev/ttyACM0` (it can flip between boots/attachments).
