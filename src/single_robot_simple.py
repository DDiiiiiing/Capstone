#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   
# TEST 2019-12-09
import rospy
import os
import threading, time
import sys,select
from geometry_msgs.msg import Twist
import tty,termios
# import msvcrt
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# ros namespace
# 컨트롤 노드와 예제파일의 노드가 일치해야만 로봇이 정상 작동
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1509"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

# 각각의 기능 실행을 위한 flag
FLAG_HOMMING = 0
FLAG_READY = 1
FLAG_MULTI_JOG_START = 2
FLAG_JOG_STOP = 3
FLAG_CONTROL = -1
# 노드 강제종료 발생 시 실행
# 셧다운 메시지 출력 + moveon 정지 명령어 실행
def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0


def getKey():
   
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# 모니터링 데이터를 출력하는 함수
# 로봇 제어기로부터 모니터링 데이터가 수신되면 주기적으로 출력
# 모니터링 데이터에는 현재 위치, 속도, IO 상태 등이 포함. 
def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state           : %d" % (msg.robot_state))
        print("  robot_state_str       : %s" % (msg.robot_state_str))
        print("  actual_mode           : %d" % (msg.actual_mode))
        print("  actual_space          : %d" % (msg.actual_space))
        print("  current_posj          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        print("  current_velj          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_velj[0],msg.current_velj[1],msg.current_velj[2],msg.current_velj[3],msg.current_velj[4],msg.current_velj[5]))
        print("  joint_abs             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.joint_abs[0],msg.joint_abs[1],msg.joint_abs[2],msg.joint_abs[3],msg.joint_abs[4],msg.joint_abs[5]))
        print("  joint_err             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.joint_err[0],msg.joint_err[1],msg.joint_err[2],msg.joint_err[3],msg.joint_err[4],msg.joint_err[5]))
        print("  target_posj           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.target_posj[0],msg.target_posj[1],msg.target_posj[2],msg.target_posj[3],msg.target_posj[4],msg.target_posj[5]))
        print("  target_velj           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.target_velj[0],msg.target_velj[1],msg.target_velj[2],msg.target_velj[3],msg.target_velj[4],msg.target_velj[5]))    
        print("  current_posx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))
        print("  current_velx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_velx[0],msg.current_velx[1],msg.current_velx[2],msg.current_velx[3],msg.current_velx[4],msg.current_velx[5]))
        print("  task_err              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.task_err[0],msg.task_err[1],msg.task_err[2],msg.task_err[3],msg.task_err[4],msg.task_err[5]))
        print("  solution_space        : %d" % (msg.solution_space))
        sys.stdout.write("  rotation_matrix       : ")
        for i in range(0 , 3):
            sys.stdout.write(  "dim : [%d]"% i)
            sys.stdout.write("  [ ")
            for j in range(0 , 3):
                sys.stdout.write("%d " % msg.rotation_matrix[i].data[j])
            sys.stdout.write("] ")
        print ##end line
        print("  dynamic_tor           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.dynamic_tor[0],msg.dynamic_tor[1],msg.dynamic_tor[2],msg.dynamic_tor[3],msg.dynamic_tor[4],msg.dynamic_tor[5]))
        print("  actual_jts            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_jts[0],msg.actual_jts[1],msg.actual_jts[2],msg.actual_jts[3],msg.actual_jts[4],msg.actual_jts[5]))
        print("  actual_ejt            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_ejt[0],msg.actual_ejt[1],msg.actual_ejt[2],msg.actual_ejt[3],msg.actual_ejt[4],msg.actual_ejt[5]))
        print("  actual_ett            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_ett[0],msg.actual_ett[1],msg.actual_ett[2],msg.actual_ett[3],msg.actual_ett[4],msg.actual_ett[5]))
        print("  sync_time             : %7.3f" % (msg.sync_time))
        print("  actual_bk             : %d %d %d %d %d %d" % (msg.actual_bk[0],msg.actual_bk[1],msg.actual_bk[2],msg.actual_bk[3],msg.actual_bk[4],msg.actual_bk[5]))
        print("  actual_bt             : %d %d %d %d %d " % (msg.actual_bt[0],msg.actual_bt[1],msg.actual_bt[2],msg.actual_bt[3],msg.actual_bt[4]))
        print("  actual_mc             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_mc[0],msg.actual_mc[1],msg.actual_mc[2],msg.actual_mc[3],msg.actual_mc[4],msg.actual_mc[5]))
        print("  actual_mt             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_mt[0],msg.actual_mt[1],msg.actual_mt[2],msg.actual_mt[3],msg.actual_mt[4],msg.actual_mt[5]))

        #print digital i/o
        sys.stdout.write("  ctrlbox_digital_input : ")
        for i in range(0 , 16):
            sys.stdout.write("%d " % msg.ctrlbox_digital_input[i])
        print ##end line
        sys.stdout.write("  ctrlbox_digital_output: ")
        for i in range(0 , 16):
            sys.stdout.write("%d " % msg.ctrlbox_digital_output[i])
        print
        sys.stdout.write("  flange_digital_input  : ")
        for i in range(0 , 6):
            sys.stdout.write("%d " % msg.flange_digital_input[i])
        print
        sys.stdout.write("  flange_digital_output : ")
        for i in range(0 , 6):
            sys.stdout.write("%d " % msg.flange_digital_output[i])
        print
        #print modbus i/o
        sys.stdout.write("  modbus_state          : " )
        if len(msg.modbus_state) > 0:
            for i in range(0 , len(msg.modbus_state)):
                sys.stdout.write("[" + msg.modbus_state[i].modbus_symbol)
                sys.stdout.write(", %d] " % msg.modbus_state[i].modbus_value)
        print

        print("  access_control        : %d" % (msg.access_control))
        print("  homming_completed     : %d" % (msg.homming_completed))
        print("  tp_initialized        : %d" % (msg.tp_initialized))
        print("  mastering_need        : %d" % (msg.mastering_need))
        print("  drl_stopped           : %d" % (msg.drl_stopped))
        print("  disconnected          : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

# 컨트롤 노드로부터 모니터링 데이터 메시지를 반복적으로 subscribe하기 위한 함수
# spin 명령어를 통해 주기적으로 호출되고 있음
def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    
def thread_publish_jog(): # Publish multi-jog 
    # rospy.is_shutdown 명령어를 반복문의 조건에 삽입하여 ros노드가 정상적으로 작동하고 있다면 계속해서 퍼블리시하도록 구현
    while not rospy.is_shutdown():
        # 플래그 컨트롤이 조그 스타트를 가리키고 있다면 멀티조그에 대한 퍼블리시 게시
        if FLAG_CONTROL == FLAG_MULTI_JOG_START:
           # if (jog_target) != pre_jog_target:
           # 조그 타겟은 좌표계에서 멀티조그를 위한 벡터값을 포함한 리스트
           # 조그를 베이스 좌표계 기준으로 수행할 것이므로 레퍼런스를move_reference_base로 지정
           # 속도는 위에서 설정한 jog_velocity 지정
            jog_multi.publish(jog_target, MOVE_REFERENCE_BASE, JOG_VELOCITY)
        # 해당 스레드를 10ms마다 반복
        rospy.sleep(0.01)
        

if __name__ == "__main__":
    
    rospy.init_node('single_robot_simple_py')
    rospy.on_shutdown(shutdown)
    #set_robot_mode  = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_mode', SetRobotMode)
    # thread_subscriber 함수를 스레드로 설정하여 메세지 콜백을 실행할 수 있도록 설정
    t1 = threading.Thread(target=thread_publish_jog)
    t1.daemon = True 
    t1.start()
    
    # 로봇 정지 명령어를 발행하기 위한 pub_stop 변수
    # stop 명령어는 일반적인 로봇 기능과 비동기적으로 작동해야 하므로 토픽으로 설정
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)   
    settings = termios.tcgetattr(sys.stdin)
    # set_robot_mode로 로봇 모드를 autonomous(자동)로 변경
    # 메뉴얼 모드에서는 감속 모드로 동작하므로 원활한 사용을 위하여 자동 모드로 설정
    # autonomous 모드로 설정되면 로봇 led가 흰색으로 점등
    ## 수동 모드(manual) 상태면 로봇 led가 푸른색으로 점등
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    

    # 로봇 사용을 위한 pos list들이 설정된 모습
    # 다양한 pos 변수들 사용 가능
    set_velx(30,20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60,40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    velx=[50, 50]
    accx=[100, 100]

    p1= posj(0,0,0,0,0,0)                    #joint
    p2= posj(-90.0, 90.0, 90.0, 0.0, 0.0, 0.0) #joint

    x1= posx(400, 500, 800.0, 0.0, 180.0, 0.0) #task
    x2= posx(400, 500, 500.0, 0.0, 180.0, 0.0) #task

    c1 = posx(559,434.5,651.5,0,180,0)
    c2 = posx(559,434.5,251.5,0,180,0)


    q0 = posj(0,0,0,0,0,0)
    q1 = posj(10, -10, 20, -30, 10, 20)
    q2 = posj(25, 0, 10, -50, 20, 40) 
    q3 = posj(50, 50, 50, 50, 50, 50) 
    q4 = posj(30, 10, 30, -20, 10, 60)
    q5 = posj(20, 20, 40, 20, 0, 90)
    qlist = [q0, q1, q2, q3, q4, q5]

    x1 = posx(600, 600, 600, 0, 175, 0)
    x2 = posx(600, 750, 600, 0, 175, 0)
    x3 = posx(150, 600, 450, 0, 175, 0)
    x4 = posx(-300, 300, 300, 0, 175, 0)
    x5 = posx(-200, 700, 500, 0, 175, 0)
    x6 = posx(600, 600, 400, 0, 175, 0)
    xlist = [x1, x2, x3, x4, x5, x6]


    X1 =  posx(370, 670, 650, 0, 180, 0)
    X1a = posx(370, 670, 400, 0, 180, 0)
    X1a2= posx(370, 545, 400, 0, 180, 0)
    X1b = posx(370, 595, 400, 0, 180, 0)
    X1b2= posx(370, 670, 400, 0, 180, 0)
    X1c = posx(370, 420, 150, 0, 180, 0)
    X1c2= posx(370, 545, 150, 0, 180, 0)
    X1d = posx(370, 670, 275, 0, 180, 0)
    X1d2= posx(370, 795, 150, 0, 180, 0)


    seg11 = posb(DR_LINE, X1, radius=20)
    seg12 = posb(DR_CIRCLE, X1a, X1a2, radius=21)
    seg14 = posb(DR_LINE, X1b2, radius=20)
    seg15 = posb(DR_CIRCLE, X1c, X1c2, radius=22)
    seg16 = posb(DR_CIRCLE, X1d, X1d2, radius=23)
    b_list1 = [seg11, seg12, seg14, seg15, seg16] 

    # 노드가 종료되기 전까지 반복적으로 움직이도록 구현
    # p1 , p2 좌표를 movej 명령어를 이용하여 반복 작동
    while not rospy.is_shutdown():
        #movej(p2, vel=100, acc=100)
        #movejx(x1, vel=30, acc=60, sol=0)
        #movel(x2, velx, accx)
        #movel(x1, velx, accx)
        #movej(p1, vel=100, acc=100)
       # movec(c1, c2, velx, accx)
       # movesj(qlist, vel=100, acc=100)
       # movesx(xlist, vel=100, acc=100)
       # move_spiral(rev=9.5,rmax=20.0,lmax=50.0,time=20.0,axis=DR_AXIS_Z,ref=DR_TOOL)
       # move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
       # moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)
       

       key = getKey()
       if key == '1' : FLAG_CONTROL = FLAG_READY
       elif key == '2' : FLAG_CONTROL == FLAG_HOMMING
       
       if FLAG_CONTROL == FLAG_HOMMING: # 호밍 : 원점, 기계적 홈 자세로 이동
            movej([0,0,0,0,0,0], 60, 30)
            FLAG_CONTROL = -1
       elif FLAG_CONTROL == FLAG_READY: # 레디 : 3,5축 90도 이동 후 플래그 컨트롤 초기화
            #rospy.loginfo("READY")
            movej([0,0,90,0,90,0], 60, 30)
            movej([0,0,0,0,0,0], 60, 30)     
            FLAG_CONTROL = -1      

       elif FLAG_CONTROL == FLAG_JOG_STOP: # 조그 스탑 : 로봇 정지를 위한 명령어를 퍼블리시하고 플래그 컨트롤 초기화
            jog_multi.publish([0,0,0,0,0,0], MOVE_REFERENCE_BASE, 0)        
            FLAG_CONTROL = -1








    print('good bye!')
