import serial
import time
import threading


def parse_note_and_handle_serial(note, port='COM4', baudrate=115200):
    initial_markings = note[0]
    speed = initial_markings["speed"]
    base = initial_markings["base"]
    scale = initial_markings["scale"]
    data_list = note[1:]
    # Calculate delay time based on speed
    delay_time = 1 / (speed / 60)

    try:
        with serial.Serial(port, baudrate, dsrdtr=False, rtscts=False, timeout=1) as ser:
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
            time.sleep(0.5)
            ser.write(("set %d %d\n" % (base, scale)).encode('ascii'))
            time.sleep(0.5)
            for text, delay in data_list:
                if text == ' ' or text == '0':  # Space or empty string for delay only
                    print(f"[Delay] {delay}")
                    time.sleep(delay * delay_time)
                else:
                    if delay > 0:
                        ser.write(('p ' + text + '\n').encode('ascii'))
                        print(f"[Send] '{text}' with delay {delay}")
                    time.sleep(abs(delay) * delay_time)
            print("All data Sent!")

    except serial.SerialException as e:
        print(f"Serial connection failed: {e}")
    except Exception as e:
        print(f"Runtime error: {e}")

