#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
import socket
from std_msgs.msg import Int32
import json

code_book = {"go": '6', "stop": '1', "yellow": '2', "slow": '3', "quick": '4', 'sidewalk_slow':'5'}
sidewalk_ymax_thresh = 700
sidewalk_slow_flag = 0 # 0 正常， 1 减速模式, 2 等待模式
sidewalk_time = 0

# 车道线辅助有关阈值
fixline_xmin_thresh = 25
fixline_xmax_thresh = 240

# 限速禁速牌
speed_limit_thresh = 2
speed_unlimit_thresh = 0
speed_limit_flag = 0
speed_unlimit_flag = 0
limit_dx_thresh = 200

# 黄灯相关
#yellow_thresh = 150
yellow_flag = 0

# 红灯相关
green_flag = 0
red_time = 0

sidewalk_stop_flag = 0
sidewalk_flag = 0




def My_decode(recv_data, code_book):
    global sidewalk_slow_flag
    global sidewalk_time
    send_data = 'niubi' # 车的动作
    yellow_data = 50 # 黄灯控制方向数据
#    print(recv_data)
    # 数据处理


    if '}{' in recv_data: # 出现'{}{}'的问题，解决方案为获取第一个
        recv_data = recv_data.split('}')[0]+'}'
    # if 'recv_data != '404':
    #     data = json.loads(recv_data)'
    if 'zzz' not in recv_data:
        data = json.loads(recv_data)
    else:
        data = 'zzz'

#    print(data)    
    global yellow_flag
#    global yellow_time

    #　黄灯
    if 'yellow_back' in data and (data['yellow_back'] >= 0.009): # 黄灯开始操作的阈值
        yellow_flag += 1
    else:
        yellow_flag = 0
    if yellow_flag > 1:
        send_data = code_book['yellow']


#限速解限速
    global speed_limit_flag
    global speed_unlimit_flag
    global speed_limit_thresh
    global speed_unlimit_thresh
    try:
        if ('speed_limit' in data) and (data['speed_limit'] < 420):
            speed_limit_flag += 1
        else:
            speed_limit_flag = 0
        if speed_limit_flag > speed_limit_thresh:
            send_data = code_book['slow']
    except TypeError:
        print('zheshixindecuowu',data['speed_limit'])

    # if ('speed_unlimit' in data) and ((640 - data['speed_unlimit'])>0):
    if ('speed_unlimit' in data) and (data['speed_unlimit'] < 400):
        speed_unlimit_flag += 1
#        print(data['speed_unlimit'])
    else:
        speed_unlimit_flag = 0
    if speed_unlimit_flag > speed_unlimit_thresh:
        send_data = code_book['quick']

    # 车灯
    global green_flag
    global red_time
    if 'green_go' in data:
        green_flag += 1
    else:
        green_flag = 0
    if green_flag > 2:
        send_data = code_book['go']
    
    if 'red_stop' in data:
        send_data = code_book['stop']
    

    # 人行道
    global sidewalk_flag
    if 'side_walk' in data and (650 > data['side_walk'] > 500):
        sidewalk_flag += 1
    else:
        sidewalk_flag = 0
    if sidewalk_flag >= 2:
        send_data = code_book['sidewalk_slow']
    
    global sidewalk_stop_flag 
    if 'side_walk' in data and (data['side_walk'] > 660):
        sidewalk_stop_flag += 1
    else:
        sidewalk_stop_flag = 0
    if sidewalk_stop_flag >= 2:
        send_data = code_book['stop']
    return send_data, yellow_data

def talker():
    host='192.168.2.111'
    port=7777
    
    msg_traffic_light=0
#    fix = 0
    car_data = '5'
#    help_data = '0'
    track = 50
#    sidewalk_track = 50
    pub1 = rospy.Publisher("/traffic_light", Int32 , queue_size=10)
#    pub2 = rospy.Publisher("/side_walk", Int32, queue_size=10)
    pub3 = rospy.Publisher("/stop_vel", Int32, queue_size=10)
#    pub4 = rospy.Publisher('/sidewalk_vel', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True) 

    rate = rospy.Rate(20) # 20hz
    
    
#    print('accept') 
    # print(sidewalk_track)
    #s.send('1'.encode('utf-8'))
    print('connecting')
    while not rospy.is_shutdown(): 
        try:
            s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            s.connect((host,port))            
            data = s.recv(1024)
            print(data)
        except ValueError:
            print('chucuole!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ',data)
        car_data, yellow_data = My_decode(data,code_book)
        # 接收到的信息为一个字符串的        print(stop_vel)
        # s.send(data.encode('utf-8'))
        #socket传送来的数据进行判断speed = 40
        #此处以“红灯停，绿灯行”为例，写下逻辑。
        if car_data == '6':
            msg_traffic_light = 6# 前进
        elif car_data == '1':
            msg_traffic_light = 1 # 停止
        elif car_data == '2': 
            msg_traffic_light = 2 # 黄灯的操作
            track = yellow_data
        elif car_data == '3':
            msg_traffic_light = 3 # 减速的操作
        elif car_data == '4': 
            msg_traffic_light = 4 # 加速的操作
        elif car_data == '5':
            msg_traffic_light = 5 # 人行道停车前的减速操作
            # print(sidewalk_track)
#        print(msg_traffic_light)
        pub1.publish(msg_traffic_light)
        pub3.publish(track)
        rate.sleep()
    s.close()
    print("Hilens安全退出！")
'''
    for i in range(0,recv_dat，a.shape[0]):
        classes = recv_data[i][4]
        if classes == 6:
            data['yellow_back'] = [(recv_data[i][0] + recv_data[i][2])/2,(recv_data[i][1] + recv_data[i][3])/2]
        if classes == 4:
            data['speed_limit'] = (recv_data[i][0] + recv_data[i][2])/2
        if classes == 5:
            data['speed_unlimit'] = (recv_data[i][0] + recv_data[i][2])/2
        if classes == 0:
            data['green_go'] = (recv_data[i][0] + recv_data[i][2])/2
        if classes == 3:
            data['red_stop'] = (recv_data[i][0] + recv_data[i][2])/2
        if classes == 2:
            data['side_walk'] = (recv_data[i][1] + recv_data[i][3])/2
        
'''

'''
def yellow_track(target, car_center):
    kp = 0.2
    error = target - car_center
    control = kp * error
    # print(control)
    direction = int(50 + control)
    if direction > 100:
        direction = 100
    elif direction < 0:
        direction = 0
    # print(direction)
    return direction



        if 'yellow_back' in data:
            yellow_data = yellow_track(data['yellow_back'], 640)
        else:
            yellow_data = 50
'''
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
