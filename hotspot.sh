#!/bin/bash

echo -e "\e[32mWe need to install some packages"
echo -e "\e[0m"
sudo apt install hostapd iproute2 iw haveged dnsmasq iptables procps bash util-linux

echo""

if ! [ -x "$(command -v create_ap)" ]; then
  cd ~/Documents
  git clone https://github.com/oblique/create_ap
  cd create_ap
  sudo make install > /dev/null
fi

echo ""
echo -e "\e[32mStarting hotspot"
echo -e "\e[0m"
l="$(ls /sys/class/net/ | grep wl)"
sudo create_ap $l $l Roverr silazik123
