#!/bin/bash
# usage: phase.sh NAME "VAR=1 VAR2=1"   (empty = flags off)

NAME=$1; FLAGS=${2:-}
OUT=~/perf_ab_2026-11-26/$NAME; mkdir -p $OUT
CONF=/etc/systemd/system/x3_server.service.d/zz-perf-test.conf
{ echo "[Service]"; for f in $FLAGS; do echo "Environment=$f"; done; } > /tmp/zz.conf
echo 1 | sudo -S -p "" cp /tmp/zz.conf $CONF
echo 1 | sudo -S -p "" systemctl daemon-reload
echo 1 | sudo -S -p "" systemctl restart x3_server
echo "$FLAGS" > $OUT/flags.txt
sleep 120   # settle
START=$(date '+%Y-%m-%d %H:%M:%S')
export ROS_DOMAIN_ID=42 ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_SUPER_CLIENT=TRUE
source /opt/ros/humble/setup.bash
timeout 60 ros2 topic hz /cmd_vel_safe > $OUT/hz_cmd_vel_safe.txt 2>&1 &
timeout 60 ros2 topic hz /scan > $OUT/hz_scan.txt 2>&1 &
timeout 60 tegrastats --interval 1000 > $OUT/tegrastats.txt 2>&1 &
top -b -d 5 -n 12 -w 200 > $OUT/top.txt
wait
journalctl -u x3_server --since "$START" --no-pager > $OUT/journal.txt
echo "done $NAME"
