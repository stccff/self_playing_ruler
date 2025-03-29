import serial
import time
import threading


# List of pitch and delay data
data_list = [
    ("6", 0.5), 
    ("1.", 0.5),
    ("2.", 0.5),
    ("3.", 0.5),
    ("2.", 1),
    ("1.", 0.5), 
    ("7", 0.5),

    ("6", 0.5), 
    ("7", 1), 
    ("5", 0.5), 
    ("6", 1),
    (" ", 1),

    ("6", 1.5),
    ("1.", 0.5), 
    ("2.", 1), 
    ("2.", 1),

    ("3.", 1),
    (" ", 3),

    ("6", 0.5), 
    ("1.", 0.5),
    ("2.", 0.5),
    ("3.", 0.5),
    ("2.", 1),
    ("1.", 0.5), 
    ("7", 0.5),

    ("6", 1.5),
    ("5", 0.5), 
    ("3", 1), 
    (" ", 1),
    
    ("6", 0.5), 
    ("5", 0.5), 
    ("6", 0.5), 
    ("1.", 0.5), 
    ("2.", 0.5), 
    ("3.", 0.5), 
    ("5", 0.5), 
    ("6", 1.5),
    (" ", 3)
]

# speed = 650
speed = 1000
base = 40
scale = 0

def handle_serial_communication(data_list, port='COM4', baudrate=115200):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print("Serial port opened, starting communication...")

            # Create a thread to read serial data
            def read_serial_data():
                print("Starting to read serial data...")
                while True:
                    if ser.in_waiting > 0:
                        data = ser.readline().decode('ascii').strip()
                        print(f"[Received] {data}")

            read_thread = threading.Thread(target=read_serial_data, daemon=True)
            read_thread.start()

            # Send data
            ser.write(("set %d %d\n" % (base, scale)).encode('ascii'))
            for text, delay in data_list:
                if text.strip() == '':  # Space or empty string for delay only
                    print(f"[Delay] {delay}")
                    time.sleep(delay * speed / 1000)
                else:
                    ser.write(('p ' + text + '\n').encode('ascii'))
                    print(f"[Send] '{text}' with delay {delay}")
                    time.sleep(delay * speed / 1000)
            print("All data Sent!")

    except serial.SerialException as e:
        print(f"Serial connection failed: {e}")
    except Exception as e:
        print(f"Runtime error: {e}")

# Start serial communication
handle_serial_communication(data_list)
