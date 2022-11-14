# Issues on Windows

## Firewall
If you connect through public network, the firewall is enabled so ROS cannot send messages.

* Either change the type of network or disable firewall for the trusted network.

## Multicast
Probably multicast is not enabled by default.

* Go to Network Connections -> Your network where robot is connected
* -> Status -> Details
    * Remember the ip and mask
* -> Properties -> IPV4 -> Properties -> Advanced
    * Set the ip and mask from previous step
    * Disable "Automatic metric"
    * Set "Interface metric" to 1