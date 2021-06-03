import cv2
from multiprocessing import  Process
from multiprocessing import Array,Value
import multiprocessing
import time
class vide(object):
    def __init__(self):
        self.data=Array('s',[1])
        # self.cap=cv2.VideoCapture(0)
        self.lock = multiprocessing.Lock()

    def writer(self):
        while True:
            self.lock.acquire()
            # ret,frame=self.cap.read()
            self.data.append(1)
            print("inputing")
            print(id(self.data))
            self.lock.release()
            time.sleep(1)

    def printer(self):
        while True:
            dt=[]
            self.lock.acquire()
            print("outputing")
            print(id(self.data))
            dt=self.data
            print(dt)
            #print('here')
            self.lock.release()
            if dt!=[]:
                print(dt)
            time.sleep(0.5)


    def main_run(self):
        processes = []
        
        t1 = Process(target=self.writer,args=())
        t2 = Process(target=self.printer,args=())
        processes.append(t1)
        processes.append(t2)
        for i in processes:
            i.start()
        for i in processes:
            i.join()	

if __name__ == '__main__':
    a=vide()
    a.main_run()
