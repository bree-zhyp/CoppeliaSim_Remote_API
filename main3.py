# This small example illustrates how to use the remote API
# synchronous mode. The synchronous mode needs to be
# pre-enabled on the server side. You would do this by
# starting the server (e.g. in a child script) with:
#
# simRemoteApi.start(19999,1300,false,true)
#
# But in this example we try to connect on port
# 19997 where there should be a continuous remote API
# server service already running and pre-enabled for
# synchronous mode.
#
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

# 导入模块，如果导入失败则输出报错信息
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import sys
import math
import numpy as np
import matplotlib.pyplot as plt  
import cv2

MAX = 1000
def distance(x, y, w, h, x1, y1, x2, y2):  
    # 计算特征点连线的斜率  
    if x1 == x2:  
        slope = np.inf  # 垂直线  
    else:  
        slope = (y2 - y1) / (x2 - x1)  
      
    # 计算特征点连线的截距  
    intercept = y1 - slope * x1  
      
    # 长方形的四个顶点坐标  
    points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]  
      
    # 初始化最小距离为无穷大  
    min_distance = np.inf  
    distance_to_line = np.inf
    
    zx = (x1+x2)/2
    zy = (y1+y2)/2

    # 遍历长方形的四条边，计算交点并找到最小距离  
    for i in range(4):  
        p1, p2 = points[i], points[(i + 1) % 4]  
          
        # 如果边是水平的（y坐标相同），则交点的y坐标与边相同  
        if near(p1[1],p2[1]):  
            y_intersect = p1[1]  

            if abs(slope) < MAX and abs(slope) > 0:
                x_intersect = (y_intersect - intercept) / slope  
            elif abs(slope) > 0:
                x_intersect = (x1+x2)/2
            else:
                continue

            if min(p1[0], p2[0]) <= x_intersect <= max(p1[0], p2[0]):  
                # 计算交点与连线的距离  
                distance_to_line = np.sqrt((zx-x_intersect)**2 + (zy-y_intersect)**2)
                min_distance = min(min_distance, distance_to_line)  
          
        # 如果边是垂直的（x坐标相同），则交点的x坐标与边相同  
        elif near(p1[0],p2[0]):  
            x_intersect = p1[0]
            y_intersect = slope * x_intersect + intercept  

            if min(p1[1], p2[1]) <= y_intersect <= max(p1[1], p2[1]):  
                # 计算交点与连线的距离  
                distance_to_line = np.sqrt((zx-x_intersect)**2 + (zy-y_intersect)**2) 
                min_distance = min(min_distance, distance_to_line)  
        else:
            print('error')

        # print('distance_to_line=', distance_to_line)
        # print('min_distance=', min_distance)
      
    # 如果没有找到交点（例如，连线与长方形平行或不相交），则返回无穷大  
    if min_distance == np.inf:  
        print('distance_to_line=', np.inf)
        return np.inf  
    
    print('distance_to_line=', distance_to_line)
    # 返回最小距离  
    return min_distance

def near(x, y):
    if x == None or y == None:
        return False
    if abs(x-y) < 0.01:
        return True
    else:
        return False
    
# 必要的一些变量
current_time = -1
straighttime = -1
backwardtime = -1
cX_blue = None
cY_blue = None
cX_red = None
cY_red = None
signal = 0
time_to_back = 0
time_to_straight = 0
time_to_turn_right = 0
time_to_turn_left = 0

# 尝试关闭所有的已开启的连接，开始运行program
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections

# 连接到CoppeliaSim的远程API服务器
clientID=sim.simxStart('127.0.0.1',-3,True,True,5000,5) # Connect to CoppeliaSim

# 如果连接API成功的话
if clientID!=-1:
    print ('Connected to remote API server')

    synchronous = True # 设置为True以启用同步模式  
    sim.simxSynchronous(clientID, synchronous)

    # sensorName = 'bubbleRob_sensingNose'  # 传感器名称  
    # error, sensorHandle = sim.simxGetObjectHandle(clientID, sensorName, sim.simx_opmode_blocking)  
    # if error != sim.simx_return_ok:  
    #     print(f'Failed to get handle for sensor {sensorName}')  
    #     sim.simxFinish(clientID)  
    #     sys.exit(1)  # 退出程序

    Vision_sensor = 'Vision_sensor'  # 视觉传感器名称  
    error, Vision_sensor_Handle = sim.simxGetObjectHandle(clientID, Vision_sensor, sim.simx_opmode_blocking)  
    if error != sim.simx_return_ok:  
        print(f'Failed to get handle for sensor {Vision_sensor}')  
        sim.simxFinish(clientID)  
        sys.exit(1)  # 退出程序

    jointName1 = 'bubbleRob_leftMotor'  # 关节名称
    error1, left_jointHandle = sim.simxGetObjectHandle(clientID, jointName1, sim.simx_opmode_blocking) 
    if error1 != sim.simx_return_ok:  
        print(f'Failed to get handle for sensor {jointName1}')  
        sim.simxFinish(clientID)  
        sys.exit(1)  # 退出程序

    jointName2 = 'bubbleRob_rightMotor'
    error2, right_jointHandle = sim.simxGetObjectHandle(clientID, jointName2, sim.simx_opmode_blocking) 
    if error2 != sim.simx_return_ok:  
        print(f'Failed to get handle for sensor {jointName2}')  
        sim.simxFinish(clientID)  
        sys.exit(1)  # 退出程序
    
    before_runtimes = time.time()
    # start the simulation: 开始仿真
    sim.simxStartSimulation(clientID,sim.simx_opmode_streaming)

    # 设置目标速度  
    targetVelocity = 175/180*math.pi

    # 设定初始速度
    sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_jointHandle, targetVelocity, sim.simx_opmode_streaming)

    # Now step a few times: 仿真步进1000步
    for i in range(1,1000):
        flag = 0 # 检测是否发生碰撞
        print('------------------epoch', i, '-----------------------')
        error_code, resolution, image_data = sim.simxGetVisionSensorImage(clientID, Vision_sensor_Handle, options=0, operationMode=sim.simx_opmode_streaming) 

        if error_code == sim.simx_return_ok:  
            image_data = np.array(image_data, dtype=np.int16)+256
            image_data = np.array(image_data, dtype=np.uint8)
            image_data.resize([resolution[1], resolution[0], 3])
            image_data = np.flipud(image_data)
            
            plt.cla()
            plt.imshow(image_data)
            plt.pause(0.01)

            # 转换到HSV颜色空间  
            hsv = cv2.cvtColor(image_data, cv2.COLOR_RGB2HSV)  
            
            # 定义红色在HSV空间中的范围  
            lower_red = np.array([0, 50, 50])  
            upper_red = np.array([10, 255, 255])
            lower_blue = np.array([110, 50, 50])  
            upper_blue = np.array([130, 255, 255])    
            lower_green = np.array([40, 100, 100])   
            upper_green = np.array([80, 255, 255])  
            
            # 创建红色和蓝色区域的掩码  
            mask_red = cv2.inRange(hsv, lower_red, upper_red)  
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue) 
            mask_green = cv2.inRange(hsv, lower_green, upper_green) 
            
            # 找到红色、蓝色和绿色的区域的轮廓  
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
            
            # 计算红色和蓝色区域的中心点  
            if contours_red:  
                c_red = max(contours_red, key=cv2.contourArea)  
                M_red = cv2.moments(c_red)  
                if M_red["m00"] != 0:  
                    cX_red = int(M_red["m10"] / M_red["m00"])  
                    cY_red = int(M_red["m01"] / M_red["m00"])  
                    print(f"Red region center at step {i}: ({cX_red}, {cY_red})")  
            else:  
                # 没有找到红色轮廓  
                cX_red, cY_red = None, None  
            
            if contours_blue:  
                c_blue = max(contours_blue, key=cv2.contourArea)  
                M_blue = cv2.moments(c_blue)  
                if M_blue["m00"] != 0:  
                    cX_blue = int(M_blue["m10"] / M_blue["m00"])  
                    cY_blue = int(M_blue["m01"] / M_blue["m00"])  
                    print(f"Blue region center at step {i}: ({cX_blue}, {cY_blue})") 
            else:  
                # 没有找到蓝色轮廓  
                cX_blue, cY_blue = None, None  
            
            # 计算绿色区域的中心点（假设障碍物是最大的绿色轮廓）  
            if contours_green:  
                c_green = max(contours_green, key=cv2.contourArea)  
                # 计算边界框  
                x, y, wide, height = cv2.boundingRect(c_green)  
            else:
                x, y, wide, height = None, None, None, None

            # 计算物体中心（红色与蓝色中心的平均值）  
            car_Xcenter = (cX_red + cX_blue) / 2 if cX_red is not None and cX_blue is not None else None  
            car_Ycenter = (cY_red + cY_blue) / 2 if cY_red is not None and cY_blue is not None else None  
            
            # 定义安全距离阈值  
            SAFE_DISTANCE_THRESHOLD = 30  # 根据实际情况调整这个值  
            
            # 检查是否发生碰撞  
            if car_Xcenter is not None and x is not None and y is not None:  
                distances = distance(x,y,wide,height,cX_red,cY_red,cX_blue,cY_blue)
                print(distances)
                if distances < SAFE_DISTANCE_THRESHOLD:  
                    flag = 1
                    print("Collision detected!")  
                else:  
                    print("No collision detected.")  
            else:  
                print("Insufficient data to detect collision.")
        
        else:  
            print('Failed to retrieve image:', error_code)

        lastCmdTime = sim.simxGetLastCmdTime(clientID)
        current_time = lastCmdTime

        if flag == 1: # 检测到碰撞
            signal = 1
            time_to_straight = 1
            time_to_back = 1
            time_to_turn_right = 1
            time_to_turn_left = 1
            straighttime = 150

        if signal == 0:
            print("go straight1!")
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, targetVelocity, sim.simx_opmode_streaming)
        elif distance(x,y,wide,height,cX_red,cY_red,cX_blue,cY_blue)<40 and time_to_back == 1:
            print("go back!")
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, -targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, -targetVelocity, sim.simx_opmode_streaming)
        elif not near(cY_red,cY_blue) and time_to_turn_left == 1:
            print("turn left!")
            time_to_back = 0
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, -targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, 0, sim.simx_opmode_streaming)
        elif straighttime > 0 and time_to_straight == 1:
            print("go straigt2!")
            time_to_turn_left = 0
            straighttime = straighttime - 1
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, targetVelocity, sim.simx_opmode_streaming)
        elif not near(cX_blue,cX_red) and time_to_turn_right == 1:
            print("turn right!")
            time_to_straight = 0
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, 0, sim.simx_opmode_streaming)
        elif signal == 1:
            signal = 0
            time_to_turn_right = 0
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, targetVelocity, sim.simx_opmode_streaming)
            
        sim.simxSynchronousTrigger(clientID); # 触发同步操作

    # stop the simulation: 停止仿真
    sim.simxStopSimulation(clientID,sim.simx_opmode_streaming)

    # Now close the connection to CoppeliaSim: 关闭连接
    sim.simxFinish(clientID) 

    after_runtime = time.time()
# 如果连接API失败的话
else:
    print ('Failed connecting to remote API server')

print ('Program ended')
print('while循环持续了', after_runtime-before_runtimes)