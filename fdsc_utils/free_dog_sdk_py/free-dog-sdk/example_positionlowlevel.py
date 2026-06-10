from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.lowState import lowState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, LOW_WIFI_DEFAULTS, LOW_WIRED_DEFAULTS
from ucl.enums import GaitType, SpeedLevel, MotorModeLow
from ucl.complex import motorCmd, motorCmdArray
import time
import sys
import math
import numpy as np
from pprint import pprint

# 关节线性插值函数，用于计算初始位置到目标位置之间的线性过渡
def jointLinearInterpolation(initPos, targetPos, rate):
    # 将rate限制在0.0到1.0之间
    rate = np.fmin(np.fmax(rate, 0.0), 1.0)
    # 计算插值后的关节位置
    p = initPos*(1-rate) + targetPos*rate
    return p

# You can use one of the 3 Presets WIFI_DEFAULTS, LOW_CMD_DEFAULTS or HIGH_CMD_DEFAULTS.
# IF NONE OF THEM ARE WORKING YOU CAN DEFINE A CUSTOM ONE LIKE THIS:
#
# MY_CONNECTION_SETTINGS = (listenPort, addr_wifi, sendPort_high, local_ip_wifi)
# conn = unitreeConnection(MY_CONNECTION_SETTINGS)
# 定义关节索引映射，便于通过名称访问关节状态
d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
     'FL_0':3, 'FL_1':4, 'FL_2':5,
     'RR_0':6, 'RR_1':7, 'RR_2':8,
     'RL_0':9, 'RL_1':10, 'RL_2':11 }

# 定义停止位置和速度的阈值
PosStopF  = math.pow(10,9)
VelStopF  = 16000.0
LOWLEVEL  = 0xff
# sine波中点位置
sin_mid_q = [0.0, 1.2, -2.0]
# 时间步长
dt = 0.002
# 初始关节位置
qInit = [0, 0, 0]
# 目标关节位置
qDes = [0, 0, 0]
# sine波计数器
sin_count = 0
# 速率计数器
rate_count = 0
# 比例增益
Kp = [0, 0, 0]
# 微分增益
Kd = [0, 0, 0]

# 打印使用的库版本
print(f'Running lib version: {lib_version()}')

# 建立与机械狗的连接，使用默认的低级WiFi设置
conn = unitreeConnection(LOW_WIFI_DEFAULTS)
# 启动接收数据
conn.startRecv()

# 初始化低级命令和状态对象
lcmd = lowCmd()

# 如果需要加密命令，可以启用以下行
# lcmd.encrypt = True
lstate = lowState()

# 初始化电机命令数组
mCmdArr = motorCmdArray()
# Send empty command to tell the dog the receive port and initialize the connection
# 发送空命令以告诉机械狗接收端口并初始化连接
cmd_bytes = lcmd.buildCmd(debug=False)
conn.send(cmd_bytes)

# 获取初始数据
data = conn.getData()
for paket in data:
    print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
    # 解析数据包
    lstate.parseData(paket)
    # 打印序列号
    print(f'SN [{byte_print(lstate.SN)}]:\t{decode_sn(lstate.SN)}')
    # 打印版本信息
    print(f'Ver [{byte_print(lstate.version)}]:\t{decode_version(lstate.version)}')
    # 打印电池状态
    print(f'SOC:\t\t\t{lstate.bms.SOC} %')
    # 打印整体电压
    print(f'Overall Voltage:\t{getVoltage(lstate.bms.cell_vol)} mv') #something is still wrong here ?!
    # 打印当前电流
    print(f'Current:\t\t{lstate.bms.current} mA')
    # 打印循环次数
    print(f'Cycles:\t\t\t{lstate.bms.cycle}')
    # 打印BQ温度
    print(f'Temps BQ:\t\t{lstate.bms.BQ_NTC[0]} °C, {lstate.bms.BQ_NTC[1]}°C')
    # 打印MCU温度
    print(f'Temps MCU:\t\t{lstate.bms.MCU_NTC[0]} °C, {lstate.bms.MCU_NTC[1]}°C')
    # 打印足部力
    print(f'FootForce:\t\t{lstate.footForce}')
    # 打印估算的足部力
    print(f'FootForceEst:\t\t{lstate.footForceEst}')
    # 打印IMU温度
    print(f'IMU Temp:\t\t{lstate.imu.temperature}')
    # 打印FR_0电机模式
    print(f'MotorState FR_0 MODE:\t\t{lstate.motorState[d["FR_0"]].mode}')
    print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')

motiontime = 0 # 动作计时器

# 主循环，持续发送命令控制机械狗
while True:
    time.sleep(0.002)   # 每2毫秒循环一次
    motiontime += 1
    data = conn.getData()
    for paket in data:
        lstate.parseData(paket)
        # 每100个循环打印一次状态信息
        if motiontime % 100 == 0: #Print every 100 cycles
            print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
            print(f'SN [{byte_print(lstate.SN)}]:\t{decode_sn(lstate.SN)}')
            print(f'Ver [{byte_print(lstate.version)}]:\t{decode_version(lstate.version)}')
            print(f'SOC:\t\t\t{lstate.bms.SOC} %')
            print(f'Overall Voltage:\t{getVoltage(lstate.bms.cell_vol)} mv') #something is still wrong here ?!
            print(f'Current:\t\t{lstate.bms.current} mA')
            print(f'Cycles:\t\t\t{lstate.bms.cycle}')
            print(f'Temps BQ:\t\t{lstate.bms.BQ_NTC[0]} °C, {lstate.bms.BQ_NTC[1]}°C')
            print(f'Temps MCU:\t\t{lstate.bms.MCU_NTC[0]} °C, {lstate.bms.MCU_NTC[1]}°C')
            print(f'FootForce:\t\t{lstate.footForce}')
            print(f'FootForceEst:\t\t{lstate.footForceEst}')
            print(f'IMU Temp:\t\t{lstate.imu.temperature}')
            print(f'MotorState FR_0 MODE:\t\t{lstate.motorState[d["FR_0"]].mode}')
            print(f'MotorState FR_0 q:\t\t{lstate.motorState[d["FR_0"]].q}')
            print(f'MotorState FR_0 dq:\t\t{lstate.motorState[d["FR_0"]].dq}')
            print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')

    if( motiontime >= 0):

        # first, get record initial position
        # 动作阶段划分
        if( motiontime >= 0 and motiontime < 10):
            # 记录初始位置
            qInit[0] = lstate.motorState[d['FR_0']].q
            qInit[1] = lstate.motorState[d['FR_1']].q
            qInit[2] = lstate.motorState[d['FR_2']].q

        # second, move to the origin point of a sine movement with Kp Kd
        if( motiontime >= 10 and motiontime < 400):
            # 过渡到正弦运动的起始点
            rate_count += 1
            rate = rate_count/200.0                   # 需要计数到200
            Kp = [5, 5, 5]				# 比例增益
            Kd = [1, 1, 1]				# 微分增益
            # Kp = [20, 20, 20]
            # Kd = [2, 2, 2]

            # 计算目标位置的插值
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate)
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate)
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate)

        # last, do sine wave
        freq_Hz = 1
        # freq_Hz = 5
        freq_rad = freq_Hz * 2* math.pi
        t = dt*sin_count
        if( motiontime >= 400):
            # 进行正弦波运动
            sin_count += 1				# 频率为1Hz
            # sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0)
            # sin_joint2 = -0.9 * sin(3*M_PI*sin_count/1000.0)
            sin_joint1 = 0.6 * math.sin(t*freq_rad)
            sin_joint2 = -0.9 * math.sin(t*freq_rad)
            qDes[0] = sin_mid_q[0]
            qDes[1] = sin_mid_q[1] + sin_joint1
            qDes[2] = sin_mid_q[2] + sin_joint2


        # 设置各个电机的命令(Servo 模式:电机会根据设定的位置指令 (q) 进行精确的位置控制)
        mCmdArr.setMotorCmd(
            'FR_0',
            motorCmd(
                mode=MotorModeLow.Servo,# 设置为Servo模式
                q=qDes[0],		 # 目标位置
                dq=0,			 # 速度设为0
                Kp=Kp[0],		 # 比例增益
                Kd=Kd[0],		 # 微分增益
                tau=-0.65		 # 力矩设定值
            )
        )
        mCmdArr.setMotorCmd(
            'FR_1',
            motorCmd(
                mode=MotorModeLow.Servo,
                q=qDes[1],
                dq=0,
                Kp=Kp[1],
                Kd=Kd[1],
                tau=0.0
            )
        )
        mCmdArr.setMotorCmd(
            'FR_2',
            motorCmd(
                mode=MotorModeLow.Servo,
                q=qDes[2],
                dq=0,
                Kp=Kp[2],
                Kd=Kd[2],
                tau=0.0
            )
        )
        # 将电机命令赋值给低级命令对象
        lcmd.motorCmd = mCmdArr

        # 构建并发送命令
        cmd_bytes = lcmd.buildCmd(debug=False)
        conn.send(cmd_bytes)
