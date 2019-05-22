This code runs on Python 2.7 on a Raspberry Pi 3B running Raspbian Stretch.

NETWORK SETUP

The pi has a static IP assigned by the router, and is controlled using ssh.

The hardware UART pins of the pi are enabled as a serial console in case it could not be accessed wirelessly.


HARDWARE

The camera is an Arducam OV5647 with the ribbon cable attached to the camera slot.

The camera is placed on a 3D-printed pan-tilt mount.
Only the upper half of the mount was used.
	https://www.thingiverse.com/thing:2860638
A Tower Pro MG996R was used to pan, on the lower half of the mount.
A Tower Pro Micro Servo 9g was used to tilt, on the upper half of the mount.
We used a very strong double-sided tape to attach the servo for upper half to a flat surface which was mounted onto the lower half.

Another Tower Pro MG996R was used for the antenna mount.

An RTL-SDR dongle is plugged into one of the Pi's USB ports, and connected to the antenna with a coaxial cable.

A USB-to-Serial adapter is plugged into one of the USB ports of the Raspberry Pi, and connected at the other end to the UART pins of the MSP430.

WIRING

The three servos are connected to a 6V battery (4 AA batteries).
They share a ground with the Raspberry Pi.

The Raspberry Pi uses a separate 3000mAh battery (usually used to charge a cell phone) connected to the microUSB port.

For the camera pan-tilt mount:
	The Tower Pro MG996R is on GPIO 17.
	Tower Pro Micro Servo 9g is on GPIO 27.
For the antenna mount:
	The other Tower Pro MG996R is on GPIO 19.


INSTALLATION

Must set up your RTL-SDR dongle to be detectable through the pyrtlsdr library.
We did that by running:
	sudo apt-get install rtl-sdr
	pip install pyrtlsdr
Then we followed the instructions posted in the JammerDetect library:
	https://github.com/mikeh69/JammerDetect/blob/master/Instructions.txt
Specifically, we ran the lines:
	cd JammerDetect
	sudo cp 88-dvb-t.rules /etc/udev/rules.d/
	sudo cp blacklist-dvb-t.conf /etc/modprobe.d/
	sudo reboot

RUN THE CODE

On the raspberry pi, cd to the "raspberrypi" directory and run:
	./start_robot.sh
	
