#!/bin/bash
# script to set low latency mode on an ftdi serial device
if [ $# -lt 1 ]; then
	echo "Usage ./set_lowlatency_mod <port>"
	exit
fi

port=$1

setserial $port low_latency