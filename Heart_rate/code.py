import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import threading 
import scipy.io as scio

cap=cv2.VideoCapture(0)
global res_avg_h
res_avg_h=[]
global hsv_store
hsv_store=[]
global time_
time_=[]
has_pic=False
start_time=time.time()
lock=threading.Lock()



def read():
    global hsv_store
    global res_avg_h
    global has_pic
    global time_
    while True:
        #print("reading")
        ret,Frame=cap.read()
        # print(Frame.shape)
        cent_x=640
        cent_y=160
        shift_x=50
        shift_y=20
        Frame=np.array(Frame)
        hsv=cv2.cvtColor(Frame,cv2.COLOR_BGR2HSV)
        hsv=cv2.flip(hsv,1)
        cv2.rectangle(hsv,(cent_x-shift_x,cent_y-shift_y),(cent_x+shift_x,cent_y+shift_y),(255,0,0))
        h_avg_temp=hsv[cent_y-shift_y:cent_y+shift_y,cent_x-shift_x:cent_x+shift_x,0]
        h_avg=h_avg_temp[h_avg_temp<50].mean()
        lock.acquire()
        hsv_store=hsv
        res_avg_h.append(h_avg)
        time_.append(time.time())
        # print(h_avg)
        has_pic=True
        lock.release()
        time.sleep(1/80)
        print(len(res_avg_h))
        if len(res_avg_h)==1000:
            print('stop')
            scio.savemat('res_avg_h.mat',{'A':np.array(res_avg_h)})
            break




def main():
    global hsv_store
    global res_avg_h
    global has_pic
    threads=[]
    threads.append(threading.Thread(target=read))
    #threads.append(threading.Thread(target=show))
    for i in threads:
        i.start()
        print("entering")

    while True:
        #print("showing")
        lock.acquire()
        #print("acquired")
        has=False
        #print("here")
        if has_pic:
            temp=hsv_store
            has=True

        if len(res_avg_h)<200:
            im_h=res_avg_h
        else:
            im_h=res_avg_h[-201:-1]
        lock.release()
        #print("releasing")
        if has:
            #print(has)
            #print(temp)
            if not temp==[]:
                #print(temp)
                #print("yes!!!!")
                cv2.imshow('PicHeartRate',np.array(temp))
                cv2.waitKey(16)
        plt.ion()
        if not im_h==[]:
            plt.clf()
            plt.plot(im_h)
            plt.show()
    for i in threads:
        i.join()


main()

