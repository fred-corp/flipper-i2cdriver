import serial
import time

# Use the port that you confirmed works with the real device
#PORT = "/dev/tty.usbmodemflip_Omskufm1"
PORT = "/dev/tty.usbserial-DM02V7KY" 

def run_sniff():
    try:
        # 1. Open with parameters matching the official library
        # rtscts=False and dsrdtr=False are crucial for bridge devices
        s = serial.Serial(
            PORT, 
            baudrate=115200, 
            timeout=0.5, 
            rtscts=False, 
            dsrdtr=False
        )
        
        # 2. Force DTR/RTS high (standard for waking up these bridges)
        s.dtr = True
        s.rts = True
        
        # 3. Purge everything (CLI noise, old prompts)
        s.reset_input_buffer()
        s.reset_output_buffer()
        time.sleep(0.1)

        print(f"--- Connected to {PORT} ---")

        # ECHO TEST
        test_byte = 0x55
        print(f"Sending Echo: 'e' + {hex(test_byte)}")
        s.write(b'e' + bytes([test_byte]))
        
        resp = s.read(1)
        if resp:
            print(f"RECEIVED ECHO: {resp.hex()}")
        else:
            print("RECEIVED ECHO: TIMEOUT")

        # STATUS TEST
        print("\nSending Status: '?'")
        s.write(b'?')
        
        # Read until we see the closing bracket ']' or timeout
        status_resp = s.read(80) 
        if status_resp:
            print(f"RECEIVED STATUS: {status_resp.decode('ascii', errors='replace')}")
        else:
            print("RECEIVED STATUS: TIMEOUT")

        s.close()

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    run_sniff()
