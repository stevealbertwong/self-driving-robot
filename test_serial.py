## http://www.instructables.com/id/Raspberry-Pi-Arduino-Serial-Communication/
## https://maker.pro/raspberry-pi/tutorial/how-to-connect-and-interface-raspberry-pi-with-arduino
## In your Raspberry Pi interface, be sure to enable Serial and I2C in PiConfig.


import serial

ser = serial.Serial('/dev/ttyACM0',9600)
s = [0]

while True:
	# ser.write("left", 100)
	ser.write(1, 200)

# while True:
# 	read_serial=ser.readline()
# 	s[0] = str(int (ser.readline(),16))
# 	print s[0]
# 	print read_serial


