# Python远程控制VREP仿真
机制：Remote API

官方介绍：
Remote API 是其他应用程序与 CoppeliaSim 连接的几种方式之一。 它允许 CoppeliaSim 和外部应用程序（即运行在不同进程或不同机器上的应用程序）之间的通信，是跨平台的，支持服务调用（即阻塞调用），以及双向数据流。

目前三种框架：ZeroMQ-based，legacy remote API，B0-based

## 关于 Legacy remote API

支持语言：C/C++，Python，Java，Matlab/Octave，Lua

远程 API 函数通过socket（或者通过共享内存）与 CoppeliaSim 交互。 

同步/异步工作模式（默认异步）
- 同步模式：VREP端每一次仿真步骤（或循环）都与远程端同步，即，VREP会等待远程端的触发信号，收到信号后才进行下一个仿真步骤。
- 异步模式：VREP端的仿真步骤按照设置的dt间隔进行，不与远程端同步。编程相对简单。

可以远程控制仿真（打开场景，开始、暂停、停止仿真等）

## 实验环境

操作系统：Windows 11 Version 23H2

Python版本：Python 3.11.4 ('base':conda)

## 配置文件

1、确定你的python源文件所在目录，即工作目录。

2、Remote API 功能需要工作目录中有以下三个文件：
```bash
A: sim.py
B: simConst.py
C: remoteApi.dll
```
A和B的位置：
```bash
C:\ProgramFiles\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\python\python\
```
C的位置：
```bash
C:\ProgramFiles\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\lib\lib\Windows
```
把三个文件复制到工作目录。

3、可以正常调用 sim.py 中提供的接口函数与VREP通信了，例如调用 sim.simxStart 等。

## 运行测试
在运行代码之前先使用 CoppeliaSim 打开 scene ([coppelia_bubbleRob.ttt](coppelia_bubbleRob.ttt)) 

可以调用下面的程序进行步进仿真。
```bash
python simpleSynchronousTest.py
```

执行 main.py 函数即可调用远程 API 实现 bubbleRob 小车的直角避障。
```bash
python main.py
```
## 运行结果展示
<img src="figure/video-online-video-cuttercom.gif" alt="运动示意">

## 作者
[Yupeng Zhang](https://github.com/SYSU-Zhangyp)

