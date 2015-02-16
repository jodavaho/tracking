#!/bin/bash
pdir=$(pwd)
cd `rospack find tagloc`
if [ -f 'tagloc_log' ]
then
	echo "Clearing" >> .old_log
	date >> .old_log
	cat tagloc_log >> .old_log
	rm tagloc_log
fi
cd "$pdir"
