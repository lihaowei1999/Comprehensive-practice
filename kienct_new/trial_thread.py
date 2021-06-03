import threading
import time

global data
data=[]
def writer():
    global data

    d=0
    while True:
        lock.acquire()
        data.append(d)
        d=d+1
        lock.release()
        time.sleep(1)

def printer():
    global data
    while True:
        lock.acquire()
        print(data)
        lock.release()
        time.sleep(5)


def main():
    threads = []
    
    t1 = threading.Thread(target=writer,args=())
    t2 = threading.Thread(target=printer,args=())
    threads.append(t1)
    threads.append(t2)
    for i in threads:
        i.start()
    for i in threads:
        i.join()	#将线程加入到主线程中

if __name__ == '__main__':
    lock = threading.Lock()
    count = 0
    main()

