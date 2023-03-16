#!/bin/sh
make
while true
do
	ps -ef | grep "./diaggrabpro" | grep -v "grep"
if [ "$?" -eq 1 ]
then
	./diaggrabpro -c ./data_service.cfg -logsize 1 -start -ftpipaddr 10.20.27.66 -username gwj -passwd gwj
	echo "process has been restarted!"
else
	echo "process already started!"
fi
	sleep 1
done
