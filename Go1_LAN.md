# Connecting the Go1 to the Internet

## 18-11-2024: Failed attempt (with Bull Dog)

1. Connect robot to Deakin LAN
2. Get robot's MAC address (`ip a`) and register robot as wired device on Deakin network
3. Edit `/etc/dhcpcd.conf`: add name servers for `eth0` (`10.128.164.31` and `10.64.164.31`) and add router (`10.150.176.1`)
4. Run `sudo dhclient eth0 -v`

**Result:** `ping 8.8.8.8` works, `ping 128.184.237.77` (IP address for `deakin.edu.au`) works - so it can connect to the outside Internet, but DNS resolution isn't working. `/etc/resolv.conf` says that nameserver is `127.0.0.1` instead of anything set by us nor DHCP.

**TODO:** get name server set up and working
