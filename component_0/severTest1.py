# @Time    : 2021/10/8 下午5:18
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : severTest1.py
# @Software: PyCharm
# TCP服务端程序
import socket
import threading
import time


def tcplink(sock, addr, is_work_flag):
    # print('Accept new connection from %s:%s...' % addr)
    # sock.send(b'Welcome!')
    recv_num = 0
    while True:
        data = sock.recv(1)
        # time.sleep(1)
        # 表示有障碍物
        if data.decode("utf-8") == '1':
            # 有障碍物
            is_work_flag[1] = 1
        if data.decode('utf-8') == "0":
            is_work_flag[1] = 0
        if not data or data.decode('utf-8') == 'exit':
            break
        # sock.send(('Hello, %s!' % data.decode('utf-8')).encode('utf-8'))
        print("is_work_flag=", is_work_flag, recv_num, time.time())
        recv_num += 1
    sock.close()
    print('Connection from %s:%s closed.' % addr)


def s_server(is_work_flag: list):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 监听端口:
    s.bind(('127.0.0.1', 9999))
    s.listen(5)
    # print('Waiting for connection...')
    while True:
        # 接受一个新连接:
        sock, addr = s.accept()
        # 创建新线程来处理TCP连接:
        t = threading.Thread(target=tcplink, args=(sock, addr, is_work_flag))
        t.start()


def c_client():
    c_freq = 20  # Hz
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 建立连接:
    s.connect(('127.0.0.1', 9999))

    # 接收欢迎消息:
    # print(s.recv(1024).decode('utf-8'))
    for i in range(10):
        for data in [b'0', b'1']:
            # 发送数据:
            s.send(data)
            print("发送数据%s", data, time.time())
            # print(s.recv(1024).decode('utf-8'))
            time.sleep(1 / c_freq)

    # s.send(b'exit')
    time.sleep(10)
    s.close()


if __name__ == '__main__':
    is_work_flag = [0]
    t_s = threading.Thread(target=s_server, args=(is_work_flag,))
    # s_server()
    t_c = threading.Thread(target=c_client)
    t_s.start()
    t_c.start()
    t_s.join()
