#!/usr/bin/env python
#coding=utf-8
import rospy
#倒入自定义的数据类型
import time
from std_msgs.msg import Int32
from std_msgs.msg import Float64
#from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import threading

HighSpeed = 100

is_Hilens_Online = 1
speed=20      # SPEED (0～100之间的值)
direction=50  # 0-LEFT-50-RIGHT-100 (0-49:左转，50:直行，51～100:右转)
gear=2        # 1 - DRIVE, 2 - NEUTRAL, 3 - PARK, 4 - REVERSE
                # 1:前进挡 2:空挡 3:停车挡 4:倒挡 0:急刹车

# 全局变量       
lane_vel = 50                   # 小车转弯值0-100,50为中间 
traffic_light_data= 100
lid_vel = 50
motorspeed = 0
mile_stone = 0
lidobj = False
park_done=False
lid_dis = 1000
stop_vel = 50
if_cross = 0
speed_high = 50
speed_low = 20 
# 阻塞监听话题
def thread_job():        
    rospy.spin()
# 巡线反馈
def lanecallback(msg):  
    global lane_vel
    lane_vel = msg.data
#雷达转角反馈
def lidcallback(msg):    
    global lid_vel
    lid_vel = msg.data
# 雷达避障反馈
def lidobj_callback(msg):
    global lidobj
    lidobj = msg.data
# 交通灯识别反馈
def lightcallback(msg):   
    global traffic_light_data, is_Hilens_Online
    traffic_light_data = msg.data
    is_Hilens_Online = 1
# 黄灯停止反馈
def stop_vel_callback(msg):
    global stop_vel, is_Hilens_Online
    stop_vel = msg.data
    is_Hilens_Online = 1
#停车结束反馈
def park_callback(msg):
    global park_done
    park_done = msg.data
# 里程计反馈
def milestone_callback(msg):
    global mile_stone
    mile_stone = msg.data
# 小车状态控制
def carCtr(_gear, _speed, _direction):
    global gear
    global speed 
    global direction
    gear = _gear
    speed = _speed
    direction = _direction


middle_targetspeed = 0
middle_flag = 0
def speed_slowdown(targetSpeed):
    global middle_targetspeed
    global middle_flag
    if middle_flag == 0:
        middle_flag = 1
        if motorspeed > (targetSpeed + 20)*100:
            middle_targetspeed = motorspeed/100 - 20
            return middle_targetspeed
        else:
            middle_targetspeed = targetSpeed
            return middle_targetspeed
    if middle_flag == 1:
        if motorspeed < (middle_targetspeed + 1)*100:
            middle_flag = 0
        else:
            return middle_targetspeed

def kinematicCtrl():
    
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。

    pub1 = rospy.Publisher('/bluetooth/received/manul', Int32 , queue_size=10)
    pub2 = rospy.Publisher('/auto_driver/send/direction', Int32 , queue_size=10)
    pub3 = rospy.Publisher('/auto_driver/send/speed', Int32 , queue_size=10)
    pub4 = rospy.Publisher('/auto_driver/send/gear', Int32 , queue_size=10)
#    pub5 = rospy.Publisher('/soundRequest', Int32 , queue_size=10)  # 0不叫，1滴滴
    pub6 = rospy.Publisher('/gameStage', Int32, queue_size = 1)


    manul=0       # 0 - Automatic(自动); 1 - Manual (手动操控)
    global speed,gear,direction,is_Hilens_Online
    is_Hilens_Online_flag = 0
    game_stage = 0  # 用于记录比赛行程阶段,0为初始位置等待绿灯出发,1为绿灯出发
    start_time = 0
    start_milestone = 0
#    didi_pub = 2
    game_start_time = 0

    rospy.init_node('kinematicCtrl', anonymous=True)        # 初始化节点kinematicCtrl
    
    add_thread = threading.Thread(target = thread_job)     # 阻塞循环接收节点
    add_thread.start()
    
    rate = rospy.Rate(30) # Hz
    rospy.Subscriber("/lane_vel", Int32, lanecallback)         # 订阅巡线转角信息
    rospy.Subscriber("/traffic_light", Int32, lightcallback)  # 订阅交通灯信息
    rospy.Subscriber("/stop_vel", Int32, stop_vel_callback)  # 订阅黄灯停止信息   
    rospy.Subscriber("/lidar_angle", Int32, lidcallback)             # 订阅雷达转角信息
    rospy.Subscriber('/has_obs', Bool, lidobj_callback)         # 订阅雷达避障信息 
    rospy.Subscriber("/ActualMilestone", Float64, milestone_callback)   # 订阅里程计信息
    rospy.Subscriber("/park", Bool,park_callback)
    
    while not rospy.is_shutdown():    
        # 此处以“红灯停、绿灯行”为例，写下逻辑。
        # 绿灯9，停止1，黄灯2，禁速3，解禁速4，快接近人行道5
        # USE (traffic_light_data)
        # TO CHANGE: GEAR, DIRECTION AND SPEED.
#        print(mile_stone)
        # 
        if is_Hilens_Online == 1:
            is_Hilens_Online = 2
            is_Hilens_Online_flag = 0
        elif is_Hilens_Online == 2:
            is_Hilens_Online = 0
        elif is_Hilens_Online == 0:
            is_Hilens_Online_flag += 1
            if is_Hilens_Online_flag > 5:
                print("Hilens离线！！！！！！！！！！！！")

        if game_stage == 0:    # 开始阶段，未检测到绿灯停止，绿灯进入第一阶段
            print('waiting')
            carCtr(0, 0, 50)
            if traffic_light_data == 6:  # 绿灯亮起
#                didi_pub = 1
#                game_start_time = time.time()
                game_stage = 1
                print("绿灯亮起")
        if game_stage == 1:  # 直走x米后，开始第二阶段
                carCtr(1, speed_high, 48)   
                if (mile_stone - start_milestone) > 1:
                    print("进入跑道！")
                    game_stage = 2  
        if game_stage == 2:   
            carCtr(1, speed_high, lane_vel)
            if traffic_light_data == 3:
                print("刹车！！！")
                start_time = time.time()
                game_stage = 16
        if game_stage == 16:   
            carCtr(0, 0, lane_vel)
            if (time.time() - start_time) > 0.01:
                print("提速")
                start_milestone = mile_stone
                game_stage = 3
        if game_stage == 6:
            carCtr(1, 30, lane_vel + lid_vel)
            if 2.6 > (mile_stone - start_milestone) > 2.5:
                print("进入桥洞")
#                carCtr(1, 20, lane_vel)
            if 5.2 > (mile_stone - start_milestone) > 5.1:
                print("出桥洞")
                start_milestone = mile_stone
                game_stage = 4        
        if game_stage == 3:  
            carCtr(1, 10, lane_vel)
            print(traffic_light_data)
            print(mile_stone - start_milestone)
            if (mile_stone - start_milestone) > 2.2:
                #traffic_light_data == 4
                print("到达解限速杆！速度为2千米每小时")
                game_stage = 6
#            if (time.time() - start_time) > 1.5:
#                game_stage = 7 ################################################
#                print("超时！交通灯未识别，假设通过限速杆！")

        if game_stage == 4:  # 检测是否第一次到达斑马线，到达斑马线进行进入第三阶段
            carCtr(1, 40, lane_vel)
#            print(traffic_light_data)
            if traffic_light_data == 5:  
                print("检测到斑马线，减速！！")
                game_stage = 8
        if game_stage == 8:
            carCtr(1, 20, lane_vel + 2)
#            print(traffic_light_data)
            if traffic_light_data == 1:
                print("准备停车！！")
#                didi_pub = 1
                start_time = time.time()
                game_stage = 9
#                carCtr(1, 30, lane_vel)
        if game_stage == 9:
            carCtr(1, 20, lane_vel)
            if (time.time() - start_time) > 1.5:
                start_time = time.time()
                print("人行道前停止")
                game_stage = 5 
        if game_stage == 5:  # 停车x秒，保证出现停车的动作，然后进入第四阶段
            carCtr(0, 0, lane_vel)
            if (time.time() - start_time) > 1:
                start_milestone = mile_stone
                print("上桥前准备")
                game_stage = 15
        if game_stage == 15:
            carCtr(1, 50, lane_vel + 3)
            print(mile_stone - start_milestone)
            if (mile_stone - start_milestone) > 4.8:
                #5.1
                print("上桥")
                game_stage = 12
        if game_stage == 12:
            carCtr(1, 40, 37 + lid_vel)
            print(lid_vel)
            print(mile_stone - start_milestone)
            if (mile_stone - start_milestone) > 11:
                start_time = time.time()
                print("开始点刹")
                game_stage = 13
        if game_stage == 13:
            carCtr(0, 0, 47)
            if (time.time() - start_time) > 0.2:
                print("开始下桥")
                game_stage = 14
        if game_stage == 14:
            carCtr(1, 30, lane_vel)
            if (mile_stone - start_milestone) > 16.5:
            #15.5
                print("左转弯结束")
                game_stage = 7
#        if game_stage == 17:
#            carCtr(1, 30, lane_vel + 15)
#            print(mile_stone - start_milestone)
#            if (mile_stone - start_milestone) > 20:
#                print("有傻逼,停车！")
#                game_stage=7    
        
        if game_stage == 7:
            carCtr(1, 40, lane_vel)
            #print(mile_stone - start_milestone)
            if lidobj == True:
                print("停车！")
                game_stage=11
        if game_stage == 11:
            carCtr(0, 0, lane_vel)
            if lidobj == False:
                print("没了！")
                game_stage=10
        if game_stage == 10:
            carCtr(1,40, lane_vel)
            if traffic_light_data == 2:
                print("看到黄灯了！！！")
#                carCtr(0, 0, 50)
                start_time = time.time()
                print("停车")
                game_stage = 99
        if game_stage == 99:
            carCtr(0, 0, 47)
            if (time.time() - start_time) > 0.5:
                print("直行")
                start_time = time.time()
                game_stage = 103

#倒车
        if game_stage == 103:
            carCtr(4, 40, 47)
            if park_done == True:#直行车道轴距底线3.7m
                print("左转")
                start_time = time.time()
                game_stage = 100

        if game_stage == 100:
            carCtr(4, 40, 15)
            if (time.time() - start_time) > 2.0:
                print("右转")
                start_time = time.time()
                game_stage = 102
        if game_stage == 102:
            carCtr(4, 40, 82)
            if (time.time() - start_time) > 2.0:
                print("停车结束")
                game_stage = 101
        if game_stage == 101:
            carCtr(0,0,50)
            game_stage=999
        pub1.publish(manul)
        pub2.publish(direction)
        pub3.publish(speed)
        pub4.publish(gear)
#        pub5.publish(didi_pub)
        pub6.publish(game_stage)
#        didi_pub=0
        rate.sleep()
    for i in range(1,5):
        pub3.publish(0)
    print('over')

if __name__ == '__main__':
    kinematicCtrl()

