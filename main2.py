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

# 必要的一些变量
detected_wall_time = backward_start_time = -1
backward_end_time = turning_start_time = -1
turning_end_time = straight_start_time = -1
straight_end_time = reset_direction_time = -1
reset_direction_end_time = -1
current_time = -1

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

    sensorName = 'bubbleRob_sensingNose'  # 传感器名称  
    error, sensorHandle = sim.simxGetObjectHandle(clientID, sensorName, sim.simx_opmode_blocking)  
    if error != sim.simx_return_ok:  
        print(f'Failed to get handle for sensor {sensorName}')  
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

    # Now step a few times: 仿真步进1000步
    for i in range(1,1000):
        # 设定初始速度
        sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, right_jointHandle, targetVelocity, sim.simx_opmode_streaming)

        # 获取上一个命令的时间以及延迟时间
        lastCmdTime = sim.simxGetLastCmdTime(clientID)
        current_time = lastCmdTime

        # 使用blocking模式读取接近传感器的数据  
        result, detected, position, _, _ = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)  

        # 检查结果是否表示成功  
        if result == sim.simx_return_ok:  
            # 如果detected为True，则表示传感器检测到了物体  
            if detected:  
                print(f"Sensor detected an object at position: {position}")  
                detected_wall_time = backward_start_time = current_time
                backward_end_time = turning_start_time = current_time + 200
                turning_end_time = straight_start_time = current_time + 5/8/targetVelocity*4*math.pi*1000 + 200
                straight_end_time = reset_direction_time = current_time + 5/8/targetVelocity*4*math.pi*1000 + 200 + 0.5/(0.08*math.pi*targetVelocity)*4*math.pi*1000
                reset_direction_end_time = current_time + 10/8/targetVelocity*4*math.pi*1000 + 200 + 0.5/(0.08*math.pi*targetVelocity)*4*math.pi*1000
            else:  
                print("Sensor did not detect any object")  
        else:  
            print(f"Failed to read proximity sensor data: {result}")

        # print("current_time= ", current_time, "detected_wall_time=", detected_wall_time, "backward_end_time=", backward_end_time)
        # print("turning_end_time= ", turning_end_time, "straight_end_time= ", straight_end_time, "reset_direction_end_time= ", reset_direction_end_time)
        if current_time >= detected_wall_time and current_time >= backward_end_time and current_time >= turning_end_time and current_time >= straight_end_time and current_time >= reset_direction_end_time:
            print("go straight1!")
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, targetVelocity, sim.simx_opmode_streaming)
        elif backward_end_time >= current_time:
            print("go back!")
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, -targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, -targetVelocity, sim.simx_opmode_streaming)
        elif turning_end_time >= current_time:
            print("turn left!")
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, -targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, 0, sim.simx_opmode_streaming)
        elif straight_end_time >= current_time:
            print("go straigt2!")
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, targetVelocity, sim.simx_opmode_streaming)
        elif reset_direction_end_time >= current_time:
            print("turn right!")
            sim.simxSetJointTargetVelocity(clientID, left_jointHandle, targetVelocity, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, right_jointHandle, 0, sim.simx_opmode_streaming)
        else:
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