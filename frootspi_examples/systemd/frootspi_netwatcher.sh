#!/bin/bash

ip monitor link |
while read -r line; do
  if [[ $line == *'state UP'* ]]; then
    # wait for ip address assign
    for i in {1..30}; do
      sleep 1
      if [[ $(ip addr show wlan0) == *'inet '* ]]; then
        echo "Network is up. Restarting robot_launch.service. "
        sudo systemctl restart robot_launch.service
        break
      fi
    done
  elif [[ $line == *'state DOWN'* ]]; then
    echo "Network is down."
  fi
done
