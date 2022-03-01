#!/usr/bin/env python

from __future__ import print_function

import threading

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs import Int8

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
X,Y movement:
   q    w    e
   a    s    d
   z    x    c
---------------------------
Z movement:
        r
        f
---------------------------
0 : nothing
1 : home position
2 : zero position

CTRL-C to quit
"""

moveBindings = {
    'q':(1,1,0),
    'w':(1,0,0),
    'e':(1,-1,0),
    'a':(0,1,0),
    's':(0,0,0),
    'd':(0,-1,0),
    'z':(-1,1,0),
    'x':(-1,0,0),
    'c':(-1,-1,0),
    'r':(0,0,1),
    'f':(0,0,-1),
}

commBindings = {
    '0':0,
    '1':1,
    '2':2    
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Int8, queue_size = 1)
        self.x = 0
        self.y = 0
        self.z = 0
        self.comm = 0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, comm):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.comm = comm
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0)
        self.join()

    def run(self):
        axis = Int8()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            axis.data.x = self.x
            axis.data.y = self.y
            axis.data.z = self.z
            axis.data.comm = self.comm

            self.condition.release()

            # Publish.
            self.publisher.publish(axis)

        # Publish stop message when thread exits.
        axis.data.x = 0
        axis.data.y = 0
        axis.data.z = 0
        axis.data.comm = 0
        self.publisher.publish(axis)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('capstone_teleop_xyz')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    comm = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z)

        print(msg)
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                comm = commBindings[key]

                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0:
                    continue
                x = 0
                y = 0
                z = 0
                comm = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, comm)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
