#!/bin/sh

# shutdown_all_robots.sh
# ロボット全台のRaspiをシャットダウンします
# 
# Usage:
# $ ./shutdown_all_robots.sh

cd `dirname $0`
mkdir -p logs

IP_ADDR_LIST="192.168.0.100 192.168.0.101 192.168.0.102 192.168.0.103 192.168.0.104 192.168.0.105 192.168.0.106 192.168.0.107 192.168.0.108 192.168.0.109 192.168.0.110 192.168.0.111"

DATE=`date '+%Y%m%d%H%M%S'`
for ip_addr in ${IP_ADDR_LIST}; do
    ./shutdown.sh ${ip_addr} > logs/shutdown_${ip_addr}_${DATE} &
done