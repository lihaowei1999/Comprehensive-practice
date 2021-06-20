
import socket


import os
import time
filename=r"D:\kienct_new\asset\skele\new25.mat"
fname_1,fname_2=os.path.split(filename)
filesize=str(os.path.getsize(filename))
print(filesize)
client_addr=("192.168.249.159",4000)
sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.connect(client_addr)
command=sock.recv(1024)
print(command)
if command.decode("gbk") == "send":
    
    print("sending file name")
    sock.send(fname_2.encode("gbk"))
    time.sleep(0.001)
    sock.send(filesize.encode("gbk"))
    time.sleep(0.001)
    print("sending files")
    with open(filename,"rb") as fi:
        transfered_data=0
        while True:
            file_data=fi.read(1024)
            if file_data:
                sock.send(file_data)
                transfered_data=transfered_data+1024
                print("sending %s : %s "%(str(transfered_data),filesize))
            else:
                print("transfered done")
                break
    sock.close()