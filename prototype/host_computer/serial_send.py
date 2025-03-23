import serial
import time


# 音高与延时数据列表
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

speed = 650

def send_serial_data(data_list, port='COM4', baudrate=115200):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            for text, delay in data_list:
                if text.strip() == '':  # 空格或空字符串仅延时
                    print(f"[延时] {delay}")
                    time.sleep(delay * speed / 1000)
                else:
                    ser.write((text + '\n').encode('ascii'))
                    print(f"[发送] '{text}' 后延时 {delay}")
                    time.sleep(delay * speed / 1000)
            print("所有数据发送完成！")
    except serial.SerialException as e:
        print(f"串口连接失败: {e}")
    except Exception as e:
        print(f"运行时错误: {e}")

send_serial_data(data_list)  # 使用默认COM4和115200波特率