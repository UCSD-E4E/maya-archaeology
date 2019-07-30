#!/bin/bash

export DISPLAY=:0

if [[ `ps -A | grep x11vnc | awk -f " " '{print $1}'` == "" ]];
then
	x11vnc -nomodtweak -bg -reopen -forever -shared -display :0 -ncache 10
fi
