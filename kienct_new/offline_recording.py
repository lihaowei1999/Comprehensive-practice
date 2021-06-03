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
import numpy as np
import time
import tqdm
if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread
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
        


        
    def clean_up(self):
        while True:
            self.lock.acquire()
            if len(self.color_frame)>200:
                self.color_frame=self.color_frame[-201:-1]
                self.xyd_data=self.xyd_data[-201:-1]
                self.xyz_data=self.xyz_data[-201:-1]
                self.frame_time=self.frame_time[-201:-1]
            self.lock.release()
            time.sleep(1/20)



    def monitor(self):
        while True:
           # print("monitoring")
            self.lock.acquire()
            frame_this=[]
            if not len(self.color_frame)==0:
                frame_this=self.color_frame[-1]
                xyd_this=self.xyd_data[-1]
            self.lock.release()
            if not frame_this==[]:
                for i in range(25):
                    cv2.circle(frame_this,(int(xyd_this[i,0]),int(xyd_this[i,1])),5,(255,0,0),4)
                cv2.imshow("show pic",frame_this[0:-1:2,0:-1:2,:])
                cv2.waitKey(1)
            time.sleep(1/10)






    def save_data(self):
        #print("tring to connect to host")
        #self.sock_up.connect(self.client_addr_up)
        #self.sock_down.connect(self.client_addr_down)
        i=160 # i indicate the file order
        while True:

            i=i+1

            self.lock.acquire()
            if len(self.xyz_data)>60:
                self.xyz_for_network=self.xyz_data[-61:-1]
                self.time_for_network=self.frame_time[-61:-1]
            path=r'D:\kienct_new\asset\record\falingtest\rubish'+str(i)+'.mat'
            io.savemat(path,{"xyzdata":np.array(self.xyz_for_network),'time':np.array(self.time_for_network),'xyddata':np.array(self.xyd_data)})
            self.lock.release()
            print(path)
            time.sleep(1)




    def run(self):
        # -------- Main Program Loop -----------

        flag_1=0
        xyd=[]
        time_run=[]
        depth_storage=[]
        
        csp_type = _ColorSpacePoint * np.int(1920 * 1080)
        csp = ctypes.cast(csp_type(), ctypes.POINTER(_DepthSpacePoint))
        
        runtime =time.time()
        
        while not self._done:
            time_this_frame=time.time()
            skelet_this_frame=np.zeros((25,3))



            has_frame=False 
            while not has_frame:
                if self._kinect.has_new_color_frame():
                    frame = self._kinect.get_last_color_frame()
                    # self.draw_color_frame(frame, self._frame_surface)
                    # framec需要回传
                    framec = np.reshape(frame,(1080, 1920, 4))
                    #cv2.imshow('rgb',framec)
                    #cv2.waitKey(2)
                    flag_frame=True
                    frame = None
                    has_frame=True
                
            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()
                
            if self._kinect.has_new_depth_frame():
                frameD=self._kinect.get_last_depth_frame()
                framed=self._kinect._depth_frame_data
                # frameD要回传
                frameD=np.reshape(frameD,(424, 512))         
                # cfrm=frameD/30
                # cfrm=cfrm.astype(np.uint8)
                flag_1=1
            
            
            
            
            depth_pos_this_frame=np.zeros((25,2))
            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for j in range(0, self._kinect.max_body_count):
                    body=self._bodies.bodies[j]
                    if body.is_tracked:
                        # print('tracked')
                        joints=body.joints
                        joint_points=self._kinect.body_joints_to_color_space(joints)
                       # joint_points_depth=self._kinect.body_joints_to_depth_space(joints)
                        
                        for i in range(0,25):
                            #print(i)
                            try:
                                
                                skelet_this_frame[i,0]=int(joint_points[i].x)
                                skelet_this_frame[i,1]=int(joint_points[i].y)
                                #print("****")
                                #print(skelet_this_frame[i,0])
                                #print(skelet_this_frame[i,1])
                                #print("***")
                            except:
                                skelet_this_frame[i,0]=0
                                skelet_this_frame[i,1]=0
                                #print('rgb reading error')
                        csp_type = _ColorSpacePoint * np.int(1920 * 1080)     
                        csp = ctypes.cast(csp_type(), ctypes.POINTER(_DepthSpacePoint))
                        self._kinect._mapper.MapColorFrameToDepthSpace(ctypes.c_uint(512*424),framed,ctypes.c_uint(1920*1080),csp)
                        #print("csp initialized")
                        for i in range(0,25):
                            try:
                                #print(i)
                                #print("tring on y")
                                node=int(skelet_this_frame[i,1]*1920+skelet_this_frame[i,0])
                                #print(skelet_this_frame[i,0])
                                #print(skelet_this_frame[i,1])
                                #print(node)
                                if node > 2073600:
                                    continue
                                if math.isinf(csp[node].y) or np.isnan(csp[node].y) : 
                                    #print('error y')
                                    skelet_this_frame[i,2]=0
                                else:
                                    #print("enter on y")
                                    depth_pos_this_frame[i,1]=csp[node].y
                                    #print(csp[node].y)

                                #print("tring on y")
                                
                                
                                if math.isinf(csp[node].x) or np.isnan(csp[node].x): 
                                    #print('error y')
                                    skelet_this_frame[i,2]=0
                                else:
                                    #print("enter on y"AAAAA)
                                    depth_pos_this_frame[i,0]=csp[node].x
                                    #print(csp[node].x)
                                    
                                #print("tring on depth")
                                
                                
                                if math.isinf(frameD[int(depth_pos_this_frame[i,1]),int(depth_pos_this_frame[i,0])]) or np.isnan(frameD[int(depth_pos_this_frame[i,1]),int(depth_pos_this_frame[i,0])]): 
                                    #print('error on depth')
                                    skelet_this_frame[i,2]=0
                                else:
                                    #print(int(depth_pos_this_frame[i,1]))
                                    #print(int(depth_pos_this_frame[i,0]))
                                    skelet_this_frame[i,2]=frameD[int(depth_pos_this_frame[i,1]),int(depth_pos_this_frame[i,0])]
                                    #print(skelet_this_frame[i,2])
                                    
                            except:
                                skelet_this_frame[i,2]=0
                        break
            flag_1=0

            xyz_this_frame=np.zeros((25,3))
            for i in range(25):
                A=np.array([[1060.569619726209,0,skelet_this_frame[i,0]],[0,1060.734317902011,skelet_this_frame[i,1]],[0,0,1]])
                B=np.array([[974.1790718952418],[548.5458090862412],[1]])*(-skelet_this_frame[i,2])
                if abs(np.linalg.det(A))>=1e-6:
                    try:
                        C=np.linalg.inv(A)@B/1000
                        xyz_this_frame[i,0]=C[0,0]
                        xyz_this_frame[i,1]=C[1,0]
                        xyz_this_frame[i,2]=skelet_this_frame[i,2]/1000
                    except:
                        xyz_this_frame[i,0]=0
                        xyz_this_frame[i,1]=0
                        xyz_this_frame[i,2]=0
                else:
                    xyz_this_frame[i,0]=0
                    xyz_this_frame[i,1]=0
                    xyz_this_frame[i,2]=0



            self.lock.acquire()
            #self.xyd_data=np.append(self.xyd_data,skelet_this_frame)
            self.xyd_data.append(skelet_this_frame)
            #self.frame_time=np.append(self.frame_time,time_this_frame)
            self.frame_time.append(time_this_frame)
            self.xyz_data.append(xyz_this_frame)
            self.color_frame.append(framec)
            self.lock.release()
            #self._clock.tick(60)
            time.sleep(1/80)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        # path=r'D:\kienct_new\asset\new.mat'
        # io.savemat(path,{"xyddata":np.array(xyd),'time':np.array(time_run)})
        #cv2.destroyAllWindows()
        pygame.quit()


    def run_all(self):
        threads = []
        threads.append(threading.Thread(target=self.run,args=()))
        threads.append(threading.Thread(target=self.save_data,args=()))
        threads.append(threading.Thread(target=self.monitor,args=()))
        threads.append(threading.Thread(target=self.clean_up,args=()))
        for i in threads:
            i.start()
        for i in threads:
            i.join()	#将线程加入到主线程中


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime()
game.run_all()     