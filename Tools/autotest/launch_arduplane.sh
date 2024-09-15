#!/bin/bash

for i in $(seq 1 $1); do
	I="$(($i-1))"
	echo "Running instance $I"
	port=$((14551+$I*10))
	gnome-terminal --title="pilot $i" --working-directory=$HOME/Desktop/ardupilot/ArduPlane -e "/home/ubuntu/Desktop/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --instance=$I --sysid=$i --out=127.0.0.1:$port
" &
sleep 3
done


masters=""
for i in $(seq 1 $1); do
	I="$(($i-1))"
	
	port=$((14550 + $I*10))
	masters="${masters} --master=127.0.0.1:$port"
done

echo $masters

#mavproxy.py --master=127.0.0.1:14551 --master=127.0.0.1:14561 --master=127.0.0.1:14571 --out=127.0.0.1:14550

mavproxy.py ${masters} --map #--console --load-module horizon  # --load module swarm # 

