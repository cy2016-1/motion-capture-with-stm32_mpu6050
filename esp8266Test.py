import socket
import threading

class Buffer:
    def __init__(self):
        self.data_buffer = ""
        self.lock = threading.Lock()

    def add_data(self, data):
        with self.lock:
            self.data_buffer += data

    def read_data(self):
        with self.lock:
            if '\n' not in self.data_buffer:
                return None  # 没有完整的数据，返回None表示没有数据可读
            else:
                data_end_index = self.data_buffer.index('\n') + 1
                data_to_read = self.data_buffer[:data_end_index]
                self.data_buffer = self.data_buffer[data_end_index:]
                return data_to_read

def read_data():
    try:
        data = client_socket.recv(4096)
    except BlockingIOError:
        print("read data failed")
        pass  # 非阻塞模式下，recv 可能会抛出 BlockingIOError，可以在这里进行其他处理





# 创建TCP socket对象
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定IP地址和端口号
server_socket.bind(('192.168.4.2', 8086))

# 监听客户端连接，队列长度为5
server_socket.listen(5)

# 接受客户端连接
client_socket, client_address = server_socket.accept()
client_socket.setblocking(False)
buffer = Buffer()

# 创建一个用于接收数据的线程
thread_data = threading.Thread(target=read_data)
thread_data.start()

try:
    while True:
        data = buffer.read_data()
        if data is not None:
            print("Reading data:", data)
except Exception as e:
    print("Error:", e)
finally:
    # 关闭套接字和线程
    client_socket.close()
    thread_data.join()
