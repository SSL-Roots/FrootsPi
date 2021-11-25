#!/bin/sh

# ssh-copy-id-for-all-robots.sh
#
# SSH鍵を転送してパスワード無しでSSHできるようにします
#
# Usage:
# $ ssh-copy-id-for-all-robots.sh

SSH_USER=ubuntu
IP_ADDR_LIST="192.168.0.100 192.168.0.101 192.168.0.102 192.168.0.103 192.168.0.104 192.168.0.105 192.168.0.106 192.168.0.107 192.168.0.108 192.168.0.109 192.168.0.110 192.168.0.111"

for ip_addr in ${IP_ADDR_LIST}; do
    ssh-copy-id ${SSH_USER}@${ip_addr}
done