#!/bin/bash
sudo apt install hostapd iproute2 iw haveged dnsmasq iptables procps bash util-linux
git clone https://github.com/oblique/create_ap
cd create_ap/
sudo make install
ifconfig
l="$(ls /sys/class/net/ | grep wl)"
sudo create_ap $l $l Roverr silazik123
