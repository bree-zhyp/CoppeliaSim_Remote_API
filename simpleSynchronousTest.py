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

# 尝试关闭所有的已开启的连接，开始运行program
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections

# 连接到CoppeliaSim的远程API服务器
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

# 如果连接API成功的话
if clientID!=-1:
    print ('Connected to remote API server')

    # enable the synchronous mode on the client: 同步模式
    sim.simxSynchronous(clientID,True)

    # start the simulation: 开始仿真
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # Now step a few times: 仿真步进1000步
    for i in range(1,1000):
        # if sys.version_info[0] == 3:
        #     input('Press <enter> key to step the simulation!')
        # else:
        #     raw_input('Press <enter> key to step the simulation!')
        sim.simxSynchronousTrigger(clientID); # 触发同步操作

    # stop the simulation: 停止仿真
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim: 关闭连接
    sim.simxFinish(clientID) 

# 如果连接API失败的话
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
