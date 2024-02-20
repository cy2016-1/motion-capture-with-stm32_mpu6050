import numpy as np
from scipy.optimize import curve_fit

# 读取txt文件
with open('data.txt', 'r') as file:
    lines = file.readlines()

# 初始化三个空列表，用于存储提取出的元素
list1_odd = []
list2_odd = []
list3_odd = []

list1_even = []
list2_even = []
list3_even = []

# 遍历行，提取元素并添加到相应的列表中
for i in range(len(lines)):
    # 将每一行的字符串按空格分割，并转换为浮点数

    # 根据行号的奇偶性将元素添加到不同的列表中
    if i % 2 == 0:  # 奇数行
        elements = [float(x) for x in lines[i][1:-2].split(',')]
        list1_odd.append(elements[0])
        list2_odd.append(elements[1])
        list3_odd.append(elements[2])
    else:  # 偶数行
        elements = [float(x) for x in lines[i][1:-2].split(',')]
        list1_even.append(elements[0])
        list2_even.append(elements[1])
        list3_even.append(elements[2])

gx = sum(list1_even) / len(list1_even) * (0xffff/500)
gy = sum(list2_even) / len(list2_even) * (0xffff/500)
gz = sum(list3_even) / len(list3_even) * (0xffff/500)


# 采集的三轴加速度计数据
axm = np.array(list1_odd)
aym = np.array(list2_odd)
azm = np.array(list3_odd)


am = np.column_stack((axm, aym, azm))

# 目标函数
def objective_function(am, a0, a1, a2, a3, a4, a5):
    return (a0 * am[:, 0] + a1)**2 + (a2 * am[:, 1] + a3)**2 + (a4 * am[:, 2] + a5)**2

# 常数项 G
G = np.ones_like(axm)

# 初始化参数估计
a0 = 1.0
a1 = 0.0
a2 = 1.0
a3 = 0.0
a4 = 1.0
a5 = 0.0

# 使用 curve_fit 进行曲线拟合
a, _ = curve_fit(objective_function, am, G, p0=[a0, a1, a2, a3, a4, a5])

calibration_factor = (250.0 / 32768.0) * (np.pi / 180.0)

aa1 = a[1] / calibration_factor
aa2 = a[3] / calibration_factor
aa3 = a[5] / calibration_factor

print("漂移参数 A", aa1, aa2, aa3)
print("拟合参数 K", a[0], a[2], a[4])
print("漂移参数 G", gx, gy, gz)

with open('data.txt', 'w') as file:
    pass  # 什么也不做，文件已经被清空

with open('offset.txt', 'a') as file:
    print(f"{{{aa1:.1f}, {aa2:.1f}, {aa3:.1f}, {a[0]:.3f}, {a[2]:.3f}, {a[4]:.3f}, {gx:.1f}, {gy:.1f}, {gz:.1f}}}", file = file)


