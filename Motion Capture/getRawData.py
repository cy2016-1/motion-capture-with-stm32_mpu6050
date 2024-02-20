import serial
import time
import numpy as np
# 打开串口，根据实际情况修改串口号和波特率
ser = serial.Serial('COM10', 1382400, timeout=1)

# 初始化变量用于累计传感器数据
acc_sum = [0.0, 0.0, 0.0]
gyro_sum = [0.0, 0.0, 0.0]
num_samples = 1000  # 可以根据需要调整采样次数

try:
    # 读取一些初始数据，以确保传感器稳定
    for _ in range(100):
        ser.readline()

    # 读取数据并计算累计和
    for _ in range(num_samples):
        line = ser.readline().decode('utf-8').strip()
        data = [float(x) for x in line.split(',')]  # 假设数据格式为 "ax,ay,az,gx,gy,gz"，可以根据实际情况修改解析方式

        acc_sum[0] += data[0]
        acc_sum[1] += data[1]
        acc_sum[2] += data[2]
        gyro_sum[0] += data[3]
        gyro_sum[1] += data[4]
        gyro_sum[2] += data[5]

    # 计算平均值，即为偏移量
    acc_offset = [acc_sum[0] / num_samples, acc_sum[1] / num_samples, acc_sum[2] / num_samples]
    gyro_offset = [gyro_sum[0] / num_samples, gyro_sum[1] / num_samples, gyro_sum[2] / num_samples]
    
    with open('data.txt', 'a') as f:
        print(acc_offset, file=f)
        print(gyro_offset, file=f)

except Exception as e:
    print(f"发生错误：{e}")

finally:
    ser.close()  # 关闭串口
