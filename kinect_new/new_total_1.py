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
        


        ## 指示变量区



        ## 

        self.client_addr_up=("192.168.249.159",5000)
        self.sock_up=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

        self.client_addr_down=("192.168.249.159",4000)
        self.sock_down=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        
        self.sock_down.connect(self.client_addr_down)
        self.sock_up.connect(self.client_addr_up)
        
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

    def get_result(self):
        #self.sock_down.connect(self.client_addr_down)
        while True:
            print("geting result")
            result=self.sock_down.recv(1024)
            print(result.decode("gbk"))
            time.sleep(1)    

    def monitor(self):
        while True:
            print("monitoring")
            self.lock.acquire()
            frame_this=[]
            if not len(self.color_frame)==0:
                frame_this=self.color_frame[-1]
                xyd_this=self.xyd_data[-1]
            self.lock.release()
            if not frame_this==[]:
                for i in range(25):
                    cv2.circle(frame_this,(int(xyd_this[i,0]),int(xyd_this[i,1])),5,(255,0,0),4)
                cv2.imshow("show pic",frame_this)
                cv2.waitKey(1)
            time.sleep(1/50)






    def save_data(self):
        #print("tring to connect to host")
        #self.sock_up.connect(self.client_addr_up)
        #self.sock_down.connect(self.client_addr_down)
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
                io.savemat(path,{"xyzdata":np.array(self.xyz_for_network),'time':np.array(self.time_for_network)})
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
            

            time.sleep(1)


    # 绘制单个骨头的函数，不需要关注
    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState
        joint1State = joints[joint1].TrackingState

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)
        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    # 绘制总体骨头的函数，不需要关注
    def draw_body(self, joints, jointPoints, color):
        # Torso
        # 这边PyKinectV2.JointType_ 代表了对应节点在数组中的位置
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
    
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);


    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()


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


            # 系统控件
            # for event in pygame.event.get(): # User did something
            #     if event.type == pygame.QUIT: # If user clicked close
            #         self._done = True # Flag that we are done so we exit this loop

            #     elif event.type == pygame.VIDEORESIZE: # window resized
            #         self._screen = pygame.display.set_mode(event.dict['size'], 
            #                                    pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

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
                                    #print("enter on y")
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
                # for i in range(0, self._kinect.max_body_count):
                #     # 几个人
                #     body = self._bodies.bodies[i]
                #     if not body.is_tracked: 
                #         continue 
                    
                #     joints = body.joints
                #     # convert joint coordinates to color space 
                #     joint_points = self._kinect.body_joints_to_color_space(joints)
                #     self.draw_body(joints, joint_points, SKELETON_COLORS[i])
                #     break
            flag_1=0

            
            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            # # h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            # # target_height = int(h_to_w * self._screen.get_width())
            # # surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            # # self._screen.blit(surface_to_draw, (0,0))
            # # surface_to_draw = None
            # # pygame.display.update()

            # # # --- Go ahead and update the screen with what we've drawn.
            # # pygame.display.flip()
            #pygame.display.update()
            # --- Limit to 60 frames per second
            xyz_this_frame=np.zeros((25,3))
            for i in range(25):
                # A=np.array([[1060.569619726209,0,skele_this_frame[i,0]],[0,1060.734317902011,skele_this_frame[i,1]],[0,0,1]])
                # B=np.array([[974.1790718952418],[548.5458090862412],[1]])*(-skelet_this_frame[i,2])
                # C=np.linalg.inv(A)@B/1000
                A=np.array([[1060.569619726209,0,999],[0,1060.734317902011,999],[0,0,1]])
                B=np.array([[974.1790718952418],[548.5458090862412],[1]])*(-1200)
                if abs(np.linalg.det(A))>=1e-6:

                    C=np.linalg.inv(A)@B/1000
                    xyz_this_frame[i,0]=C[0,0]
                    xyz_this_frame[i,1]=C[1,0]
                    xyz_this_frame[i,2]=C[2,0]
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
            #self._clock.tick(33)

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
        threads.append(threading.Thread(target=self.get_result,args=()))
        for i in threads:
            i.start()
        for i in threads:
            i.join()	#将线程加入到主线程中


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime()
game.run_all()