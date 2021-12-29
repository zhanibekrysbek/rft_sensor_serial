

1. Reduce Latency

	- change the low latency method 1
	> sudo setserial /dev/ttyUSB0 low_latency
	> sudo cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
	1 <- latency was changed.


2. Set Baud Rate

	stty -F /dev/ttyUSB1 921600


3. Give access permision to other users

	sudo chmod 777 /dev/ttyUSB0


3. Execute

	rosrun rft_sensor_serial rft_sensor_serial


4. Rosservice call ...

	Sensor 1 - C00300119
	Sensor 2 - C00300122 




