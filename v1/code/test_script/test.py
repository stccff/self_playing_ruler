import serial
import time
import threading

def send_testpos_commands(port, baudrate=115200):
    def read_serial(ser):
        while True:
            try:
                line = ser.readline()
                if line:
                    print(f"Received: {line.decode('utf-8', errors='replace').strip()}")
            except serial.SerialException:
                break

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            reader_thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
            reader_thread.start()
            for i in range(301):
                cmd = f"testpos {i}\n"
                ser.write(cmd.encode('utf-8'))
                print(f"Sent: {cmd.strip()}")
                time.sleep(0.15)
            # 等待一会儿以确保接收线程能打印完最后的消息
            time.sleep(1)
    except serial.SerialException as e:
        print(f"Serial error: {e}")

# 示例用法
send_testpos_commands('COM4', 115200)
