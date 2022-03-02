#!/usr/bin/env python

from __future__ import print_function
from timeit import repeat
import rospy
import os
import threading, time
import sys, select, termios, tty

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

msg = """
Reading from the keyboard  and Publishing to DSR
Press only one key 
---------------------------
Linear movement:
        w           r (up)
   a    s    d      f (down)
Holonomic movement:
---------------------------
         (pitch)
            u          l (roll ccw)
 (yaw)  h   j   k      ; (roll cw)

1 : home
2 : ready
anything else : stop

CTRL-C to quit
"""

l_moveBindings = {
#            f r u
        'w':(1,0,0),
        'a':(0,-1,0),
        's':(-1,0,0),
        'd':(0,1,0),
        'r':(0,0,1),
        'f':(0,0,-1),
}
a_moveBindings = {
#             w p r      
        'u':(0,0.5,0),
        'h':(-0.5,0,0),
        'j':(0,-0.5,0),
        'k':(0.5,0,0),
        'l':(0,0,-0.5),
        ';':(0,0,0.5),
}

# for single robot 
ROBOT_ID     = "dsr01"         # Robot Model & Robot ID identify namespace 
ROBOT_MODEL  = "m1509"      
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

jog_target = [0, 0, 0, 0, 0, 0]
JOG_VELOCITY = 100

FLAG_HOMMING = 0
FLAG_READY = 1
FLAG_MULTI_JOG_START = 2
FLAG_JOG_STOP = 3

FLAG_CONTROL = -1

jog_multi = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/jog_multi', JogMultiAxis, queue_size=1) # Publish multi-jog topic

#### robot control
def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        #print("  current_posx      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))

        #print("  io_control_box    : %d" % (msg.io_control_box))
        ##print("  io_modbus         : %d" % (msg.io_modbus))
        ##print("  error             : %d" % (msg.error))
        #print("  access_control    : %d" % (msg.access_control))
        #print("  homming_completed : %d" % (msg.homming_completed))
        #print("  tp_initialized    : %d" % (msg.tp_initialized))
        #print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    


def thread_publish_jog(): # Publish multi-jog 
    while not rospy.is_shutdown():
        if FLAG_CONTROL == FLAG_MULTI_JOG_START:
            # if (jog_target) != pre_jog_target:
            jog_multi.publish(jog_target, MOVE_REFERENCE_BASE, JOG_VELOCITY)
        rospy.sleep(0.01)

#### get key
def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#### main
if __name__=="__main__":
    rospy.init_node('teleop_test_py')
    rospy.on_shutdown(shutdown)
    
    t1 = threading.Thread(target=thread_publish_jog)
    t1.daemon = True 
    t1.start()
    
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)

    settings = termios.tcgetattr(sys.stdin)
    
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None
        
    rep = 0

    x = 0 
    y = 0
    z = 0
    w = 0
    p = 0
    r = 0
    status = -1 # 

    while not rospy.is_shutdown():
        #### update key input
        try:
            print(msg)
            key = getKey(key_timeout)
            if key in l_moveBindings.keys():
                x = l_moveBindings[key][0]
                y = l_moveBindings[key][1]
                z = l_moveBindings[key][2]
                FLAG_CONTROL = FLAG_MULTI_JOG_START
            elif key in a_moveBindings.keys():
                w = a_moveBindings[key][0]
                p = a_moveBindings[key][1]
                r = a_moveBindings[key][2]
                FLAG_CONTROL = FLAG_MULTI_JOG_START
            elif key == '1':
                FLAG_CONTROL = FLAG_HOMMING
            elif key == '2':
                FLAG_CONTROL = FLAG_READY

                if (rep == 14):
                    print(msg)
                rep = (rep + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already stopped.
                if key == '' and x == 0 and y == 0 and z == 0 \
                            and w == 0 and p == 0 and r == 0 and \
                            FLAG_CONTROL == FLAG_MULTI_JOG_START:
                    FLAG_CONTROL = FLAG_JOG_STOP
                    continue
                x = 0
                y = 0
                z = 0
                w = 0
                p = 0
                r = 0
                if (key == '\x03'):
                    break
            
        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        jog_target = [x, y, z, r, p, w]

        #### check key input
        if FLAG_CONTROL == FLAG_HOMMING:
            movej([0,0,0,0,0,0], 60, 30)
            FLAG_CONTROL = -1
        elif FLAG_CONTROL == FLAG_READY:
            #rospy.loginfo("READY")
            movej([0,0,90,0,90,0], 60, 30)      
            FLAG_CONTROL = -1      
        elif FLAG_CONTROL == FLAG_MULTI_JOG_START:
            print("-------------------------------")
            print("command : ", end='')
            print(jog_target)
            print("-------------------------------")
            
        elif FLAG_CONTROL == FLAG_JOG_STOP:
            jog_multi.publish([0,0,0,0,0,0], MOVE_REFERENCE_BASE, 0)        
            FLAG_CONTROL = -1

    print('good bye!')