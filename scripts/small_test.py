import serial
s = serial.Serial("/dev/tty.usbmodemflip_Omskufm1", timeout=1)
s.write(b'e\x55')
print(s.read(1).hex())
