import serial
import time
import threading


def read_serial(ser):
    while True:
        try:
            line = ser.readline()
            if line:
                print(f"Received: {line.decode('utf-8', errors='replace').strip()}")
        except serial.SerialException:
            break

def send_testpos_commands(port, baudrate=115200):
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

def send_test_magnet(port, baudrate=115200):
    """
    循环通过串口发送指定命令，每隔 interval 秒发送一次，并读取并打印串口输出。
    """
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            reader_thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
            reader_thread.start()

            while True:
                command = 'testmagnet 0 1'
                ser.write((command + '\n').encode('utf-8'))
                # print(f"Sent: {command}")
                command = 'testmagnet 1 -1'
                ser.write((command + '\n').encode('utf-8'))
                print(f"Sent: {command}")
                print("attract")
                time.sleep(2)

                command = 'testmagnet 0 -1'
                ser.write((command + '\n').encode('utf-8'))
                print("repulse")
                time.sleep(2)

                command = 'testmagnet 0 0'
                ser.write((command + '\n').encode('utf-8'))
                time.sleep(0.01)
                command = 'testmagnet 1 0'
                ser.write((command + '\n').encode('utf-8'))
                print("off")
                time.sleep(2)

    except serial.SerialException as e:
        print(f"Serial error: {e}")


def send_test_magnet_time(port, baudrate=115200):
    """
    循环通过串口发送指定命令，每隔 interval 秒发送一次，并读取并打印串口输出。
    """
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            reader_thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
            reader_thread.start()

            # 吸引
            command = 'testmagnet2 0 1 1 -1'
            ser.write((command + '\n').encode('utf-8'))
            time.sleep(8)

            # 排斥
            command = 'testmagnet2 0 1 1 1'
            ser.write((command + '\n').encode('utf-8'))
            time.sleep(0.011)

            # 关闭
            command = 'testmagnet2 0 0 1 0'
            ser.write((command + '\n').encode('utf-8'))
            print("off")

            time.sleep(2)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

# 示例用法
# send_testpos_commands('COM4', 115200)
# send_test_magnet('COM4', 115200)
send_test_magnet_time('COM4', 115200)
