import pigpio
import time
import rospy
from dream_mobile_platform.msg import EncoderMsg

import posix_ipc
import mmap
import struct

IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

LEFT_SH_MEMORY="/LEFT_SH_MEMORY"
RIGHT_SH_MEMORY="/RIGHT_SH_MEMORY"
SH_SIZE = 4096
    
class PigpioMotorControl:
    def __init__(self):
        self.pi = pigpio.pi()
        self.pi.set_PWM_frequency(ENA, 2000)
        self.pi.set_PWM_frequency(ENB, 2000)
        # Set initial states
        self.pi.write(ENA, 1)
        self.pi.write(IN1, 0)
        self.pi.write(IN2, 0)
        self.pi.write(ENB, 1)
        self.pi.write(IN3, 0)
        self.pi.write(IN4, 0)
        # Start with a duty cycle of 0
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.set_PWM_dutycycle(ENB, 0)
        print(f'{self.__class__.__name__} started')

    def forward(self, leftspeed, rightspeed):
        self.pi.write(IN1, 1)
        self.pi.write(IN2, 0)
        self.pi.write(IN3, 1)
        self.pi.write(IN4, 0)
        # Map speed (assuming 0-100 input) to PWM duty cycle (0-255)
        self.pi.set_PWM_dutycycle(ENA, int(leftspeed * 255 / 100))
        self.pi.set_PWM_dutycycle(ENB, int(rightspeed * 255 / 100))

    # When done, it's a good idea to stop the pigpio instance
    def cleanup(self):
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.set_PWM_dutycycle(ENB, 0)
        self.pi.stop()
        print(f'{self.__class__.__name__} cleaned up')

if __name__ == "__main__":
    pmc = PigpioMotorControl()
    try:
        # Try to open existing shared memory
        left_sh_memory  = posix_ipc.SharedMemory(LEFT_SH_MEMORY)
    except posix_ipc.ExistentialError:
        # If it doesn't exist, create it
        left_sh_memory = posix_ipc.SharedMemory(LEFT_SH_MEMORY, flags=posix_ipc.O_CREX, mode=0o777, size=SH_SIZE)
    # Map the shared memory to a memory view
    left_mapfile = mmap.mmap(left_sh_memory.fd, left_sh_memory.size)
    # Serialize and write the float to shared memory
    my_float = 3.141592653589793
    # ??? what does struct do
    packed_float = struct.pack('f', my_float)
    left_mapfile.write(packed_float)

    # Close the memory map and shared memory file descriptor
    left_mapfile.close()
    left_sh_memory.close_fd()

    start = time.time()
    stop = False
    LEFT_PWM = 20
    RIGHT_PWM = 20

    last_left = 0
    last_right = 0
    def cb(msg: EncoderMsg):
        global LEFT_PWM
        global RIGHT_PWM, last_left, last_right
        left, right = msg.left, msg.right
        # #TODO Remember to remove
        # print(f'Rico: {left, right}')
        # if left < last_left:
        #     LEFT_PWM = 0
        # last_left = left
        # if right < last_right:
        #     RIGHT_PWM = 0
        # last_right = right

    rospy.init_node("test_motor")
    sub = rospy.Subscriber("/dream/encoder_status", EncoderMsg, cb)

    while not rospy.is_shutdown():
        # forward(LEFT_PWM, RIGHT_PWM)
        pmc.forward(LEFT_PWM, RIGHT_PWM)
        rospy.sleep(0.04)

    # motors_cleanup()
    pmc.cleanup()
