import cv2
import threading
import time
class vide(object):
    def __init__(self):
        self.data=[]
        self.cap=cv2.VideoCapture(0)
        self.lock = threading.Lock()

    def writer(self):
        while self.cap.isOpened():
            self.lock.acquire()
            ret,frame=self.cap.read()
            self.data=frame
            #print(data)
            self.lock.release()
            time.sleep(0.005)

    def printer(self):
        while True:
            if self.data!=[]:
                print("here")
                self.lock.acquire()
                cv2.imshow("shower",self.data)
                cv2.waitKey(1)
                self.lock.release()
                time.sleep(0.01)


    def main_run(self):
        threads = []
        
        t1 = threading.Thread(target=self.writer,args=())
        t2 = threading.Thread(target=self.printer,args=())
        threads.append(t1)
        threads.append(t2)
        for i in threads:
            i.start()
        for i in threads:
            i.join()	

if __name__ == '__main__':
    a=vide()
    a.main_run()
