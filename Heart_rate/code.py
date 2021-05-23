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
##
global filter_
#filter_=[0.0099324,0.0011921,-0.016655,-0.006391,0.0061777,-0.02137,-0.024054,0.037095,0.051606,-0.0068095,0.0043572,0.065339,-0.023617,-0.20466,-0.16375,0.13223,0.31201,0.13223,-0.16375,-0.20466,-0.023617,0.065339,0.0043572,-0.0068095,0.051606,0.037095,-0.024054,-0.02137,0.0061777,-0.006391,-0.016655,0.0011921,0.0099324]
filter_=[0.020959,0.042843,0.018273,0.014349,-0.0084678,-0.0092295,0.00021913,0.016169,0.012637,-0.021264,-0.077567,-0.12294,-0.12041,-0.0538,0.055509,0.15669,0.1977,0.15669,0.055509,-0.0538,-0.12041,-0.12294,-0.077567,-0.021264,0.012637,0.016169,0.00021913,-0.0092295,-0.0084678,0.014349,0.018273,0.042843,0.020959]

## filters
global after_bp
after_bp=[]
global after_diff
after_diff=[]
global after_sq
after_sq=[]
global after_mean
after_mean=[]

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
        time_.append(time.time())
        # print(Frame.shape)
        cent_x=640
        cent_y=160
        shift_x=50
        shift_y=20
        Frame=np.array(Frame)
        hsv=cv2.cvtColor(Frame,cv2.COLOR_BGR2HSV)
        hsv=cv2.flip(hsv,1)
        h_avg_temp=hsv[cent_y-shift_y:cent_y+shift_y,cent_x-shift_x:cent_x+shift_x,0]
        try:
            h_avg=h_avg_temp[h_avg_temp<12].mean()
        except:
            h_avg=0
        cv2.rectangle(hsv,(cent_x-shift_x,cent_y-shift_y),(cent_x+shift_x,cent_y+shift_y),(255,0,0))
        lock.acquire()
        hsv_store=hsv
        res_avg_h.append(h_avg)
        # print(h_avg)
        has_pic=True
        if len(time_)>5:
            print(1/(time_[-1]-time_[-2]))
        lock.release()
        time.sleep(1/100)
        
        #print(len(res_avg_h))
        # if len(res_avg_h)==1000:
        #     print('stop')
        #     scio.savemat('res_avg_h.mat',{'A':np.array(res_avg_h)})
        #     break


# def filter():
#     global res_avg_h
#     global after_bp
#     global after_diff
#     global after_sq
#     global after_mean
#     global filter_
#     wid=16
#     while True:
#         print("filtering")
#         lock.acquire()
#         while len(after_bp) < len(res_avg_h):
#             if len(after_bp) < 32:
#                 after_bp.append(0)
#             else:
#                 # print("filter!!!!")
#                 # print(len(res_avg_h))
#                 st=len(after_bp)-32
#                 # print("st")
#                 # print(st)
#                 # print(len(res_avg_h[st:st+33]))
#                 # print(len(filter_))
#                 after_bp.append(np.dot(res_avg_h[st:st+33],filter_))
            
#         while len(after_diff)<len(after_bp):
            
#             if len(after_diff)==0:
#                 after_diff.append(0)
#             else:
#                 ll=len(after_diff)
#                 #print(after_bp)
#                 after_diff.append((after_bp[ll]-after_bp[ll-1])*30)
                
#         while len(after_sq)<len(after_diff):
#             after_sq.append(after_diff[len(after_sq)]**2)

#         while len(after_mean)<len(after_sq):
#             if len(after_mean)<wid-1:
#                 after_mean.append(0)
#             else:
#                 st=len(after_mean)-wid+1
#                 after_mean.append(np.mean(after_sq[st:st+wid]))
#         # print('out')
#         # print(after_bp)
#         # print(after_diff)
#         # print(after_sq)
#         # print(after_mean)
#         lock.release()
#         time.sleep(1/100)

def filter():
    global res_avg_h
    global after_bp
    global after_diff
    global after_sq
    global after_mean
    global filter_
    wid=16
    while True:
        #print("filtering")
        lock.acquire()
        while len(after_bp) < len(res_avg_h):
            if len(after_bp) < 32:
                after_bp.append(0)
            else:
                # print("filter!!!!")
                # print(len(res_avg_h))
                st=len(after_bp)-32
                # print("st")
                # print(st)
                # print(len(res_avg_h[st:st+33]))
                # print(len(filter_))
                after_bp.append(np.dot(res_avg_h[st:st+33],filter_))
            
                
        # while len(after_sq)<len(after_bp):
        #     after_sq.append(after_bp[len(after_sq)]**2)

        while len(after_mean)<len(after_bp):
            if len(after_mean)<wid-1:
                after_mean.append(0)
            else:
                st=len(after_mean)-wid+1
                after_mean.append(np.mean(after_bp[st:st+wid]))
        # print('out')
        # print(after_bp)
        # print(after_diff)
        # print(after_sq)
        # print(after_mean)
        lock.release()
        time.sleep(1/100)

def main():
    global hsv_store
    global res_avg_h
    global has_pic
    threads=[]
    threads.append(threading.Thread(target=read))
    threads.append(threading.Thread(target=filter))
    #threads.append(threading.Thread(target=show))
    for i in threads:
        i.start()
        print("entering")

    # display 
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
        mean_st=[0]
        if len(after_mean)<200:
            mean_st=after_mean
        else:
            mean_st=after_mean[-201:-1]

        # print("st")
        # print(len(after_bp))
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
            #plt.plot(im_h)
            plt.plot(mean_st)
            plt.show()
    for i in threads:
        i.join()


main()

