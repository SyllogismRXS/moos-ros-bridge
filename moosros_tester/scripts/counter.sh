#!/bin/bash

# Launch the ROS nodes
roslaunch moosros_tester counter.launch &
ROSLAUNCH_PID=$!

# Launch the MOOS nodes
pAntler $(rospack find moosros)/bridge.moos >& /dev/null &
PANTLER_PID=$!

uMAC $(rospack find moosros)/bridge.moos

printf "Killing all processes, safely ... \n"
kill -SIGTERM $ROSLAUNCH_PID
kill -SIGTERM $PANTLER_PID
printf "Done killing processes.   \n"


