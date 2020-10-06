#!/bin/bash
#Since 16.04, the default latency_timer value for the ftdi usb driver was changed from 1ms to 16ms. This value
#represents how long the kernel driver waits before passing data up to the user application. It significantly affects
#the speed the driver will be able to communicate with the dynamixels, resulting in very low achievable loop rates.
#To correct this, this script will change a serial port into 'low latency' mode (a latency_timer value of 1).
if [ $# -lt 1 ]; then
	echo "Usage ./set_lowlatency_mod <port>"
	exit
fi

port=$1

setserial $port low_latency