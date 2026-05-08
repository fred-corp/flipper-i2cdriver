import serial
import private.i2cdriver as i2cdriver
import time

PORT = "/dev/tty.usbmodemflip_Omskufm1"

def start_bridge():
    print("Pre-cleaning serial port...")
    # Open at a high baud rate and just read everything to clear the pipe
    with serial.Serial(PORT, baudrate=921600, timeout=0.2) as s:
        s.reset_input_buffer()
        garbage = s.read(1000)
        if garbage:
            print(f"Cleared {len(garbage)} bytes of junk from CLI.")
    
    time.sleep(0.5) # Wait for Flipper to settle

    try:
        print("Connecting to I2CDriver API...")
        # reset=False prevents the library from sending 'x', which can 
        # trigger more CLI junk if the bridge isn't 100% ready[cite: 9, 10]
        i2c = i2cdriver.I2CDriver(PORT, reset=False)
        print("Success!")
        print("Status:", i2c.getstatus())
        return i2c
    except Exception as e:
        print(f"Connection failed: {e}")
        return None

if __name__ == "__main__":
    i2c = start_bridge()
