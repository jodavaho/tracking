#!/bin/bash
#bin/server
rosparam set /tagloc/sigs .25
rosparam set /tagloc/sigb 1
rosparam set /tagloc/Cd 20
screen -dm -S tagloc rosrun tagloc server
rosrun tagloc client set 49691 -10 -40 900 0 0 800
rosrun tagloc client set 48971 -10 -40 900 0 0 850
rosrun tagloc client set 48671 -10 -40 850 0 0 900
rosrun tagloc client set 10 3 -40 900 -100 -100 900
rosrun tagloc client get 48971
rosrun tagloc client get 49691
