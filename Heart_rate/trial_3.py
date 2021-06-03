from typing_extensions import ParamSpecKwargs
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import threading 
import scipy.io as scio
from scipy.fftpack import fft,ifft
from matplotlib.pylab import mpl
from scipy.signal import find_peaks_cwt
from scipy.signal import find_peaks
import warnings
warnings.filterwarnings("ignore")

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
#filter_=[0.020959,0.042843,0.018273,0.014349,-0.0084678,-0.0092295,0.00021913,0.016169,0.012637,-0.021264,-0.077567,-0.12294,-0.12041,-0.0538,0.055509,0.15669,0.1977,0.15669,0.055509,-0.0538,-0.12041,-0.12294,-0.077567,-0.021264,0.012637,0.016169,0.00021913,-0.0092295,-0.0084678,0.014349,0.018273,0.042843,0.020959]
#filter_=[-0.12301,0.044629,0.026635,0.00056317,-0.026693,-0.039546,-0.026438,0.0051491,0.029283,0.019799,-0.024597,-0.071002,-0.071151,0.0033876,0.13334,0.25707,0.30802,0.25707,0.13334,0.0033876,-0.071151,-0.071002,-0.024597,0.019799,0.029283,0.0051491,-0.026438,-0.039546,-0.026693,0.00056317,0.026635,0.044629,-0.12301]
filter_=[0.0094165,0.0024153,0.0022291,0.0017036,0.00089208,-9.4812e-05,-0.0011126,-0.0020006,-0.0026191,-0.0028707,-0.0027308,-0.0022496,-0.0015571,-0.00083662,-0.00030001,-0.00013789,-0.00048868,-0.0013933,-0.0027867,-0.0044831,-0.0062119,-0.0076406,-0.0084434,-0.0083492,-0.007208,-0.0050187,-0.0019604,0.0016365,0.0053223,0.0086091,0.011035,0.012259,0.012113,0.010665,0.0082072,0.0052362,0.0023343,0.0001275,-0.00085458,-0.00032049,0.0017709,0.0051079,0.0090876,0.012859,0.01546,0.015964,0.013647,0.0081376,-0.0004751,-0.011597,-0.024152,-0.036695,-0.047573,-0.055151,-0.058034,-0.05529,-0.046613,-0.032401,-0.013772,0.0075585,0.029459,0.049629,0.065882,0.076423,0.080073,0.076423,0.065882,0.049629,0.029459,0.0075585,-0.013772,-0.032401,-0.046613,-0.05529,-0.058034,-0.055151,-0.047573,-0.036695,-0.024152,-0.011597,-0.0004751,0.0081376,0.013647,0.015964,0.01546,0.012859,0.0090876,0.0051079,0.0017709,-0.00032049,-0.00085458,0.0001275,0.0023343,0.0052362,0.0082072,0.010665,0.012113,0.012259,0.011035,0.0086091,0.0053223,0.0016365,-0.0019604,-0.0050187,-0.007208,-0.0083492,-0.0084434,-0.0076406,-0.0062119,-0.0044831,-0.0027867,-0.0013933,-0.00048868,-0.00013789,-0.00030001,-0.00083662,-0.0015571,-0.0022496,-0.0027308,-0.0028707,-0.0026191,-0.0020006,-0.0011126,-9.4812e-05,0.00089208,0.0017036,0.0022291,0.0024153,0.0094165]
## filters
global after_bp
after_bp=[]
global after_diff
after_diff=[]
global after_sq
after_sq=[]
global after_mean
after_mean=[]
global command

has_pic=False
start_time=time.time()
global lock
lock=threading.Lock()




def read():
    global lock
    global hsv_store
    global res_avg_h
    global has_pic
    global time_
    
    while True:
        #print("reading")
        ret,Frame=cap.read()
        tt=time.time()
        
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
        has_pic=True
        time_.append(tt)
        lock.release()
        time.sleep(1/100)

def  input():
    global command
    global lock
    while True:
        a=input()
        lock.acquire()
        command=a;
        lock.release()
        time.sleep(1)


def filter():
    global lock
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
        #带通滤波
        while len(after_bp) < len(res_avg_h):
            if len(after_bp) < 128:
                after_bp.append(0)
            else:

                st=len(after_bp)-128

                after_bp.append(np.dot(res_avg_h[st:st+129],filter_))

        #均值处理
        while len(after_mean)<len(after_bp):
            if len(after_mean)<wid-1:
                after_mean.append(0)
            else:
                st=len(after_mean)-wid+1
                after_mean.append(np.mean(after_bp[st:st+wid]))

        lock.release()
        time.sleep(1/100)

def main():
    global lock
    global hsv_store
    global res_avg_h
    global has_pic
    threads=[]
    threads.append(threading.Thread(target=read))
    threads.append(threading.Thread(target=filter))
    #threads.append(threading.Thread(target=input))
    for i in threads:
        i.setDaemon(True)

    for i in threads:
        i.start()

    while True:

        lock.acquire()
        has=False
        if has_pic:
            temp=hsv_store
            has=True



        mean_st=[]
        if len(after_mean)<200:
            mean_st=after_mean
            tm=time_[0:len(after_mean)]
            
            
        else:
            mean_st=after_mean[len(after_mean)-200:len(after_mean)]
            tm=time_[len(after_mean)-200:len(after_mean)]
            
        # print(len(time_))
        # print(len(after_mean))
        # print(len(tm))
        # print(len(mean_st))
        lock.release()
        


        plt.ion()
        if not len(tm)==0:
            plt.clf()
            # print("here")
            # print(len(tm))
            # print(len(mean_st))
            #plt.plot(30*np.linspace(0,1,num=len(fft_s)-1),np.abs(fft_s)[1:])
            plt.plot(mean_st[0:len(tm)])
            #plt.scatter([20,40,50],[12,32,12])
            if len(tm)>180:
                #pks=find_peaks_cwt(mean_st[0:len(tm)],(20+np.array(range(20)))/10)

                pks_=find_peaks(mean_st[0:len(tm)],distance=14)
                pks=pks_[0]
                #print(pks)
                if len(pks) > 4:
                    pks_st=pks[0]
                    pks_ed=pks[-1]
                    t_span=tm[pks_ed]-tm[pks_st]
                    hr_=60/(t_span/(len(pks)-1))
                    print("heart rate:  "+str(hr_))

                if not len(pks)==0:
                    
                    plt.scatter(pks,np.array(mean_st)[pks])
            plt.show()

        if has:
            if not temp==[]:
                cv2.imshow('PicHeartRate',np.array(temp))
                cv2.waitKey(1)


    # for i in threads:
    #     i.join()


main()

