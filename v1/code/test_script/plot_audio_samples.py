#!/usr/bin/env python3
import sounddevice as sd
import matplotlib.pyplot as plt
import numpy as np
import platform

if __name__ == '__main__':

    # If you got "ValueError: No input device matching", that is because your PC name example device
    # differently from tested list below. Uncomment the next line to see full list and try to pick correct one
    # print(sd.query_devices())

    fs = 48000  		# Sample rate
    duration = 2000e-3   # Duration of recording


   # check microphone device name
    print("Input devices:")
    print(sd.query_devices())


    if platform.system() == 'Windows':
        # WDM-KS is needed since there are more than one MicNode device APIs (at least in Windows)
        # device = 'Microphone (MicNode_4_Ch), Windows WDM-KS'
        device = '麦克风 (DRI Mic), Windows WDM-KS'
    elif platform.system() == 'Darwin':
        device = 'MicNode_1_Ch'
    else:
        device ='default'

    # 推荐直接用设备编号
    # device_index = 97  # 你的 MicNode_1_Ch 的编号

    try:
        myrecording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='float32', device=device)
    except Exception as e:
        print("录音失败:", e)
        exit(1)

    print('Waiting...')
    sd.wait()  # Wait until recording is finished
    print('Done!')

    print("myrecording type:", type(myrecording))
    print("myrecording shape:", myrecording.shape)
    print("myrecording dtype:", myrecording.dtype)
    # print(myrecording.T)

    time = np.arange(0, duration, 1 / fs)  # time vector
    plt.plot(time, myrecording)
    plt.xlabel('Time [s]')
    plt.ylabel('Amplitude')
    plt.title('MicNode 1 Channel')
    plt.show()
