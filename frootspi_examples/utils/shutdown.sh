#!/bin/sh

# shutdown.sh
# IPアドレスを指定して、SSHログインを行いシャットダウンします
# 一気にシャットダウンしたいときは shutdown_all_robots.sh を使ってください
#
# Usage:
# $ shutdown.sh IP_ADDRESS

SSH_USER=ubuntu

if [ -z "${1}" ]; then
    echo "IPアドレスを渡してください"
    exit 1
fi

IP_ADDR=${1}

echo "=========="
echo "PING test..."

ping -c 1 -W 1 ${IP_ADDR}

if [ $? -ne 0 ]; then
    echo "PING Failed."
    exit 2
fi
echo "PING OK!"

echo "=========="
echo "SSH Connect..."
ssh ${SSH_USER}@${IP_ADDR} <<EOC
sudo poweroff
EOC
echo "SSH Command completed"
echo "=========="

