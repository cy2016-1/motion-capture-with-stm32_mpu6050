import bluetooth

# 定义连接的地址和端口
target_address = "98:D3:02:96:5E:E4"
port = 1  # 默认端口号

# 连接到设备
sock = bluetooth.BluetoothSocket()  # 创建RFCOMM套接字对象
firstUpdate = True

def receive_message(sock):
    buffer_size = 4096
    data = b""
    
    while True:
        chunk = sock.recv(buffer_size)
        if not chunk:
            break
        data += chunk
        if b"\n" in data:
            break
    
    return data[:-3].decode('utf-8')



try:
    sock.connect((target_address, port))
    print("Connected to {} on port {}".format(target_address, port))
    
    while True:
        # 在你的代码中使用
        s = receive_message(sock)
        # print(s)
        angles = [x.split(',') for x in s.split(';')]

        try:
            for i in range(len(angles)):
                angles[i] = [float(x) for x in angles[i]]
        except ValueError as ve:
            print(f"Error converting string to float: {ve}")
        print(angles)

    # # 接收数据
    # while True:
    #     if firstUpdate:
    #         while not sock.recv(1024).decode('utf-8').endswith('\r\n'):
    #             pass
    #         firstUpdate = False
    #     # Read serial data
    #     angles = []
    #     for i in range(10):
    #         s = sock.recv(256).decode('utf-8')
    #         if s.endswith('\r\n'):
    #             s = s[:-2]
    #         angles.append([float(value) for value in s.rstrip(';').split(',')])
    #     print("Angles:", angles)
    #     print("Length:", len(angles))
        
        


except bluetooth.btcommon.BluetoothError as e:
    print("Error establishing connection:", str(e))
finally:
    # 关闭连接
    sock.close()
    print("Connection closed.")

