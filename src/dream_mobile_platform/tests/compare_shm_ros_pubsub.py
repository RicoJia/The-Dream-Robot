#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from threading import Thread
import posix_ipc
import mmap
import struct

SHM_SIZE = 4
class ShmSub:
    def __init__(self, name, cb):
        try:
            self.shm  = posix_ipc.SharedMemory(name)
        except posix_ipc.ExistentialError:
            self.shm  = posix_ipc.SharedMemory(name, flags=posix_ipc.O_CREX, size=SHM_SIZE)
        try:
            self.sem = posix_ipc.Semaphore(name)
        except posix_ipc.ExistentialError:
            self.sem = posix_ipc.Semaphore(name, flags=posix_ipc.O_CREX, initial_value=1)
        self.mmap = mmap.mmap(self.shm.fd, self.shm.size)
        self.shm.close_fd()
        self.cb = cb
        th = Thread(target=self._run, daemon=True, )
        th.start()
    def _run(self):
        while not rospy.is_shutdown():
            # self.sem.acquire()
            try:
                packed_data = self.mmap[:4]
                msg = struct.unpack('f', packed_data)[0]
                self.cb(msg)
            finally:
                # self.sem.release()
                pass
    def cleanup(self):
        self.sem.unlink()
        self.shm.unlink()

class ShmPub:
    def __init__(self, name):
        try:
            self.shm  = posix_ipc.SharedMemory(name)
        except posix_ipc.ExistentialError:
            self.shm  = posix_ipc.SharedMemory(name, flags=posix_ipc.O_CREX, size=SHM_SIZE)
        try:
            self.sem = posix_ipc.Semaphore(name)
        except posix_ipc.ExistentialError:
            self.sem = posix_ipc.Semaphore(name, flags=posix_ipc.O_CREX, initial_value=1)
        self.mmap = mmap.mmap(self.shm.fd, self.shm.size)
        self.shm.close_fd()

    def publish(self, msg):
        packed_data = struct.pack('f', msg)
        if len(packed_data) > SHM_SIZE:
            raise ValueError('Message too large')
        # self.sem.acquire()
        try:
            self.mmap[:len(packed_data)] = packed_data
        finally:
            # self.sem.release()
            pass
    def cleanup(self):
        self.mmap.close()
        self.sem.unlink()
        self.shm.unlink() 

def check_shm_pub_sub(i):
    # on rpi, 80? tmux 45; without semaphores, 80%
    def cb(num):
        #TODO Remember to remove
        print(f'Rico: {num}')
    sub = ShmSub(f'shm_{i}', cb)
    pub = ShmPub(f'shm_{i}')
    while not rospy.is_shutdown():
        pub.publish(i)
        rospy.sleep(0.02)
    sub.cleanup()
    pub.cleanup() 

#####################################################
# ROS pub/sub
#####################################################
        
def check_pub_sub(i):
    # 40%, 30% for tmux
    def cb(msg):
        #TODO Remember to remove
        print(f'Rico: {msg.data}')
    sub = rospy.Subscriber("/compare_shm{i}", Float32, cb)
    pub = rospy.Publisher("/compare_shm{i}", Float32, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(i)
        rospy.sleep(0.02)
    
if __name__ == '__main__':
    # rospy.init_node('compare_shm_ros_pubsub', anonymous=True)
    threads = [
    Thread(target=check_shm_pub_sub, args=(i,), daemon=True) for i in range(10)]
    # Thread(target=check_pub_sub, args=(i,), daemon=True) for i in range(10)]
    [t.start() for t in threads]
    # rospy.spin()
    while True:
        try:
            rospy.sleep(1)
        except KeyboardInterrupt:
            break
        
    