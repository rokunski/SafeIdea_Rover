#!/bin/bash
if test "$~/Documents/create_ap"
then
l="$(ls /sys/class/net/ | grep wl)"
sudo create_ap $l $l Roverr silazik123
else
sudo apt install hostapd iproute2 iw haveged dnsmasq iptables procps bash util-linux
cd ~/Documents
git clone https://github.com/oblique/create_ap
cd create_ap/
sudo make install
l="$(ls /sys/class/net/ | grep wl)"
sudo create_ap $l $l Roverr silazik123
fi

