#!/bin/sh

for x in 1 2 3 4 5 6 7 8 9 10
do
	./launch_drones.sh 12
	rosrun collision_avoidance cerchio_node >> collision_log.txt
	./kill_drones.sh
done
