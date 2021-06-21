import socket
import time
import os
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
path_base="/mnt/disk/lzh/socket/asset"
address = ('', 4000)
s.bind(address)
s.listen(20)
print('Waiting for connection...')
count=0
sock,addr = s.accept()
print('Accept new connection from %s:%s...' % addr)
str="send"
sock.send(str.encode("gbk"))
fname = sock.recv(1024)
print(fname)
fsize = sock.recv(1024)
print(fsize)
file_size=int(fsize.decode("gbk"))
received_size = 0
with open(fname,'wb') as fi:
    while received_size < file_size:
        data = sock.recv(1024)
        fi.write(data)
        received_size += len(data)
        print('已接收 ',received_size,' Byte')
fi.close()
sock.close()
s.close()
