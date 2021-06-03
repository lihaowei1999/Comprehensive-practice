import cv2
import socket
import os
import threading
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import math
import ctypes
import _ctypes
import pygame
import sys
from scipy import io
import scipy.io as scio
from scipy.fftpack import fft,ifft
from scipy.signal import find_peaks_cwt
from scipy.signal import find_peaks
import numpy as np
import time
import tqdm
if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread
#from typing_extensions import ParamSpecKwargs
import matplotlib.pyplot as plt
from matplotlib.pylab import mpl
import warnings
#warnings.filterwarnings("ignore")


SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]


class BodyGameRuntime(object):
    def __init__(self):
        ## pygame以及kinect初始化区
        pygame.init()
        self._clock = pygame.time.Clock()
        self._infoObject = pygame.display.Info()
        #self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
        #                                       pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
        #pygame.display.set_caption("Kinect for Windows v2 Body Game")
        self._done = False
        self._clock = pygame.time.Clock()
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body | PyKinectV2.FrameSourceTypes_Depth)
        #self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)
        self._bodies = None

        ## 线程和锁初始化
        self.lock = threading.Lock()

        self.time_start=time.time()

        ## 数据变量交换区
        ## 时间戳都保存绝对时间
        # 存储xyd数据
        self.xyd_data=[]
        # 储存视频流时间戳
        self.frame_time=[]
        # 储存xyz数据
        self.xyz_data=[]
        # 储存色彩数据，设置max帧数
        self.color_frame=[]
        # 储存ir数据，设置max帧数
        self.ir_frame=[]
        # 储存处理完的xyz数据，并已经整理成可供神经网络使用的状态
        self.xyz_for_network=[]
        self.time_for_network=[]
        # 储存神经网络的结果
        self.result_pos=[]
        # 储存神经网络结果的时间
        self.result_time=[]
        # 储存视频心率的单帧结果
        self.heart_rate_frame=[]
        
        # filter
        self.h_thr=15
        self.d_thr=5
        self.filter_=[0.0094165,0.0024153,0.0022291,0.0017036,0.00089208,-9.4812e-05,-0.0011126,-0.0020006,-0.0026191,-0.0028707,-0.0027308,-0.0022496,-0.0015571,-0.00083662,-0.00030001,-0.00013789,-0.00048868,-0.0013933,-0.0027867,-0.0044831,-0.0062119,-0.0076406,-0.0084434,-0.0083492,-0.007208,-0.0050187,-0.0019604,0.0016365,0.0053223,0.0086091,0.011035,0.012259,0.012113,0.010665,0.0082072,0.0052362,0.0023343,0.0001275,-0.00085458,-0.00032049,0.0017709,0.0051079,0.0090876,0.012859,0.01546,0.015964,0.013647,0.0081376,-0.0004751,-0.011597,-0.024152,-0.036695,-0.047573,-0.055151,-0.058034,-0.05529,-0.046613,-0.032401,-0.013772,0.0075585,0.029459,0.049629,0.065882,0.076423,0.080073,0.076423,0.065882,0.049629,0.029459,0.0075585,-0.013772,-0.032401,-0.046613,-0.05529,-0.058034,-0.055151,-0.047573,-0.036695,-0.024152,-0.011597,-0.0004751,0.0081376,0.013647,0.015964,0.01546,0.012859,0.0090876,0.0051079,0.0017709,-0.00032049,-0.00085458,0.0001275,0.0023343,0.0052362,0.0082072,0.010665,0.012113,0.012259,0.011035,0.0086091,0.0053223,0.0016365,-0.0019604,-0.0050187,-0.007208,-0.0083492,-0.0084434,-0.0076406,-0.0062119,-0.0044831,-0.0027867,-0.0013933,-0.00048868,-0.00013789,-0.00030001,-0.00083662,-0.0015571,-0.0022496,-0.0027308,-0.0028707,-0.0026191,-0.0020006,-0.0011126,-9.4812e-05,0.00089208,0.0017036,0.0022291,0.0024153,0.0094165]
        self.cent_x=960
        self.cent_y=300
        self.shift_x=100
        self.shift_y=60

        self.origin_data=[]
        self.after_bp=[]
        self.after_mean=[]
        self.wid=16
        ## 指示变量区


        ## 展示内容区
        self.heart_rate=0
        self.has_hr_display=False
        self.time_for_hr=0

        self.status=[]

        self.string_display=""
        self.has_string_display=False
        self.time_for_string=0

        self.client_addr_up=("192.168.249.159",5000)
        self.sock_up=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

        self.client_addr_down=("192.168.249.159",4000)
        self.sock_down=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        
        self.sock_down.connect(self.client_addr_down)
        self.sock_up.connect(self.client_addr_up)

        ## 
    def filter(self):
        self.lock.acquire()
        fil=self.filter_
        self.lock.release()
        wid=16
        while True:
            #print("filtering")
            self.lock.acquire()
            #带通滤波
            while len(self.after_bp) < len(self.origin_data):
                if len(self.after_bp) < 128:
                    self.after_bp.append(0)
                else:

                    st=len(self.after_bp)-128

                    self.after_bp.append(np.dot(self.origin_data[st:st+129],fil))

            #均值处理
            while len(self.after_mean)<len(self.after_bp):
                if len(self.after_mean)<wid-1:
                    self.after_mean.append(0)
                else:
                    st=len(self.after_mean)-wid+1
                    self.after_mean.append(np.mean(self.after_bp[st:st+wid]))

            self.lock.release()
            time.sleep(1/50)

    def get_result(self):
        #self.sock_down.connect(self.client_addr_down)
        while True:
            #print("geting result")
            result=self.sock_down.recv(1024)
            res=result.decode("gbk")
            #res="123"
            if len(res) > 0:
                res=int(res)
                self.lock.acquire()
                self.status.append(res)
                self.lock.release()
            time.sleep(1/3)
        
    def statistic(self):
        while True:
            self.lock.acquire()
            st=self.status
            self.lock.release()

            ## st to res

            res="hello"
            self.lock.acquire()
            self.string_display=res
            self.has_string_display=False
            self.lock.release()
            time.sleep(1)


    def clean_up(self):
        while True:
            self.lock.acquire()
            if len(self.color_frame)>200:
                self.color_frame=self.color_frame[-201:-1]
                self.xyd_data=self.xyd_data[-201:-1]
                self.xyz_data=self.xyz_data[-201:-1]
                #self.frame_time=self.frame_time[-201:-1]
            self.lock.release()
            time.sleep(1/20)

    def monitor(self):
        while True:
            self.lock.acquire()
            frame_this_=[]
            if not len(self.color_frame)==0:
                frame_this_=self.color_frame[-1]
                xyd_this=self.xyd_data[-1]

            if len(self.after_mean)<200:
                mean_st=self.after_mean
                tm=self.frame_time[0:len(self.after_mean)]
                
                
            else:
                mean_st=self.after_mean[len(self.after_mean)-200:len(self.after_mean)]
                tm=self.frame_time[len(self.after_mean)-200:len(self.after_mean)]


            if not self.has_hr_display:
                hr_this=self.heart_rate
                self.has_hr_display=True
                self.time_for_hr=time.time()
            elif time.time()-self.time_for_hr<3:
                hr_this=self.heart_rate
            else:
                hr_this=0
                

            if not self.has_string_display:
                str_this=self.string_display
                self.has_string_display=True
                self.time_for_string=time.time()
            elif time.time()-self.time_for_string<3:
                str_this=self.string_display
            else:
                str_this=""
            self.lock.release()

            plt.ion()
            #print(len(tm))
            if not len(tm)==0:
                plt.clf()
                plt.plot(mean_st[0:len(tm)])
                if len(tm)>180:

                    pks_=find_peaks(mean_st[0:len(tm)],distance=14)
                    pks=pks_[0]
                    #print(pks)
                    if len(pks) > 4:
                        pks_st=pks[0]
                        pks_ed=pks[-1]
                        t_span=tm[pks_ed]-tm[pks_st]
                        hr_=60/(t_span/(len(pks)-1))
                        print("heart rate:  "+str(hr_))
                        self.lock.acquire()
                        self.heart_rate=hr_
                        self.has_hr_display=False
                        self.lock.release()
                    if not len(pks)==0:

                        plt.scatter(pks,np.array(mean_st)[pks])
                plt.show()

            self.lock.acquire()
            lu_bound=(self.cent_x-self.shift_x,self.cent_y-self.shift_y)
            rd_bound=(self.cent_x+self.shift_x,self.cent_y+self.shift_y)
            self.lock.release()

            
            if not frame_this_==[]:
                frame_this=frame_this_[:,:,0:3].copy()
                for i in range(25):
                    cv2.circle(frame_this,(int(xyd_this[i,0]),int(xyd_this[i,1])),5,(255,0,0),4)
                cv2.rectangle(frame_this,lu_bound,rd_bound,(255,0,0),4)
                if hr_this<30:
                    hr_dis="no heart rate detected"
                else:
                    hr_dis="heart rate:"+str(hr_this)
                frame_this=cv2.putText(frame_this,hr_dis,lu_bound,cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255),3)
                frame_this=cv2.putText(frame_this,str_this,(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255),3)

                cv2.imshow("show pic",frame_this[0:-1:2,0:-1:2,])
                cv2.waitKey(1)
            time.sleep(1/50)






    def save_data(self):

        i=0 # i indicate the file order
        while True:
            want_data=self.sock_up.recv(1024)
            if want_data.decode("gbk")=="send":
                i=i+1

                self.lock.acquire()
                if len(self.xyz_data)>60:
                    self.xyz_for_network=self.xyz_data[-61:-1]
                    self.time_for_network=self.frame_time[-61:-1]
                path=r'D:\kienct_new\asset\skele\new'+str(i)+'.mat'
                io.savemat(path,{"xyzdata":np.array(self.xyz_for_network),'time':np.array(self.time_for_network),'xyddata':np.array(self.xyd_data)})
                self.lock.release()

                file_size=str(os.path.getsize(path))
                fname_1,fname_2=os.path.split(path)
                print("sending"+fname_2)
                sz='0'*(10-len(file_size))+file_size
                fname_2="x"*(10-len(fname_2))+fname_2
                self.sock_up.send(fname_2.encode("gbk"))
                time.sleep(0.001)
                

                self.sock_up.send(sz.encode("gbk"))
                time.sleep(0.001)
                with open(path,"rb") as fi:
                    transfered_data=0
                    while True:
                        file_data=fi.read(1024)
                        if file_data:
                            self.sock_up.send(file_data)
                            transfered_data=transfered_data+1024
                            #print("sending %s : %s "%(str(transfered_data),file_size))
                        else:
                            print("transfered done")
                            break
            

            time.sleep(1/5)

    def reconstruct(self):
        while True:
            self.lock.acquire()
            xyd_to_deal=self.xyd_data[len(self.xyz_data):]
            self.lock.release()
            if len(xyd_to_deal) > 0:
                # deal
                xyz_done=[]
                for this_xyd in xyd_to_deal:
                    xyz_this_frame=np.zeros((25,3))
                    for i in range(25):
                        A=np.array([[1060.569619726209,0,this_xyd[i,0]],[0,1060.734317902011,this_xyd[i,1]],[0,0,1]])
                        B=np.array([[974.1790718952418],[548.5458090862412],[1]])*(-this_xyd[i,2])
                        if abs(np.linalg.det(A))>=1e-6:
                            try:
                                C=np.linalg.inv(A)@B/1000
                                xyz_this_frame[i,0]=C[0,0]
                                xyz_this_frame[i,1]=C[1,0]
                                xyz_this_frame[i,2]=this_xyd[i,2]/1000
                            except:
                                xyz_this_frame[i,0]=0
                                xyz_this_frame[i,1]=0
                                xyz_this_frame[i,2]=0
                        else:
                            xyz_this_frame[i,0]=0
                            xyz_this_frame[i,1]=0
                            xyz_this_frame[i,2]=0
                    xyz_done.append(xyz_this_frame)
                self.lock.acquire()
                self.xyz_data.extend(xyz_done)
                self.lock.release()
            time.sleep(1/60)



    def run(self):
        flag_1=0
        xyd=[]
        time_run=[]
        depth_storage=[]
        
        csp_type = _ColorSpacePoint * np.int(1920 * 1080)
        csp = ctypes.cast(csp_type(), ctypes.POINTER(_DepthSpacePoint))
        
        runtime =time.time()
        self.lock.acquire()
        h_thr=self.h_thr
        d_thr=self.d_thr
        self.lock.release()


        while not self._done:
            time_this_frame=time.time()
            skelet_this_frame=np.zeros((25,3))
            has_frame=False 
            while not has_frame:
                if self._kinect.has_new_color_frame():
                    frame = self._kinect.get_last_color_frame()
                    framec = np.reshape(frame,(1080, 1920, 4))
                    flag_frame=True
                    frame = None
                    has_frame=True
            
            wanted_rgb=np.array(framec[self.cent_y-self.shift_y:self.cent_y+self.shift_y,self.cent_x-self.shift_x:self.cent_x+self.shift_x,0:3])
            wanted_hsv=cv2.cvtColor(wanted_rgb,cv2.COLOR_BGR2HSV)[:,:,0]
            try:
                wt=wanted_hsv[wanted_hsv<h_thr]
                h_avg=wt[wt>d_thr].mean()
            except:
                #print("here")
                h_avg=0
            #print(h_avg)
            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()
                
            if self._kinect.has_new_depth_frame():
                frameD=self._kinect.get_last_depth_frame()
                framed=self._kinect._depth_frame_data
                frameD=np.reshape(frameD,(424, 512))         
                flag_1=1
            
            
            
            
            depth_pos_this_frame=np.zeros((25,2))
            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for j in range(0, self._kinect.max_body_count):
                    body=self._bodies.bodies[j]
                    if body.is_tracked:
                        joints=body.joints
                        joint_points=self._kinect.body_joints_to_color_space(joints)
                        
                        for i in range(0,25):
                            try:
                                
                                skelet_this_frame[i,0]=int(joint_points[i].x)
                                skelet_this_frame[i,1]=int(joint_points[i].y)
                            except:
                                skelet_this_frame[i,0]=0
                                skelet_this_frame[i,1]=0
                        csp_type = _ColorSpacePoint * np.int(1920 * 1080)     
                        csp = ctypes.cast(csp_type(), ctypes.POINTER(_DepthSpacePoint))
                        self._kinect._mapper.MapColorFrameToDepthSpace(ctypes.c_uint(512*424),framed,ctypes.c_uint(1920*1080),csp)
                        for i in range(0,25):
                            try:
                                node=int(skelet_this_frame[i,1]*1920+skelet_this_frame[i,0])
                                if node > 2073600:
                                    continue
                                if math.isinf(csp[node].y) or np.isnan(csp[node].y) : 
                                    skelet_this_frame[i,2]=0
                                else:

                                    depth_pos_this_frame[i,1]=csp[node].y
                                
                                
                                if math.isinf(csp[node].x) or np.isnan(csp[node].x): 
                                    skelet_this_frame[i,2]=0
                                else:
                                    depth_pos_this_frame[i,0]=csp[node].x
                                
                                
                                if math.isinf(frameD[int(depth_pos_this_frame[i,1]),int(depth_pos_this_frame[i,0])]) or np.isnan(frameD[int(depth_pos_this_frame[i,1]),int(depth_pos_this_frame[i,0])]): 
                                    skelet_this_frame[i,2]=0
                                else:
                                    skelet_this_frame[i,2]=frameD[int(depth_pos_this_frame[i,1]),int(depth_pos_this_frame[i,0])]
                            except:
                                skelet_this_frame[i,2]=0
                        break
                ''' 这部分用来做深度图像匹配的现显示
                for i in range(25):
                    for j in range(5):
                        for k in range(5):
                            cfrm[int(depth_pos_this_frame[i,1]+j),int(depth_pos_this_frame[i,0]+k)]=255
                    for j in range(10):
                        for k in range(10):
                            cfrm[100+j,200+k]=255
                        cv2.imshow('depth',cfrm)
                '''
            flag_1=0



            self.lock.acquire()
            self.xyd_data.append(skelet_this_frame)
            self.frame_time.append(time_this_frame)
            self.origin_data.append(h_avg)
            #self.xyz_data.append(xyz_this_frame)
            self.color_frame.append(framec)
            self.lock.release()
            #time.sleep(1/80)
            self._clock.tick(30)
        self._kinect.close()
        pygame.quit()


    def run_all(self):
        threads = []
        threads.append(threading.Thread(target=self.run,args=()))
        threads.append(threading.Thread(target=self.save_data,args=()))
        threads.append(threading.Thread(target=self.monitor,args=()))
        threads.append(threading.Thread(target=self.clean_up,args=()))
        threads.append(threading.Thread(target=self.reconstruct,args=()))
        threads.append(threading.Thread(target=self.filter,args=()))
        threads.append(threading.Thread(target=self.get_result,args=()))
        threads.append(threading.Thread(target=self.statistic,args=()))
        #threads.append(threading.Thread(target=self.get_result,args=()))
        for i in threads:
            i.setDaemon(True)
            i.start()
            
        for i in threads:
            i.join()	#将线程加入到主线程中


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime()
game.run_all()     