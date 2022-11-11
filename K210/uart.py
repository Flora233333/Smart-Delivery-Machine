# uart - By: charles - 周三 11月 2 2022

import sensor, image, time, math, utime                           # 导入感光元件模块 sensor 机器视觉模块 image 跟踪运行时间模块 time 数学函数模块 math
import machine                                              # 导入模块 machine
from machine import UART                                    # 从 machine 模块中导入 双向串行通信模块 UART
from fpioa_manager import fm                                # 从 fpioa_manager 模块中导入 引脚注册模块 fm
from Maix import GPIO                                       # 从 Maix 模块中导入 引脚模块 GPIO


fm.register(10, fm.fpioa.UART1_TX, force=True)
fm.register(9, fm.fpioa.UART1_RX, force=True)

fm.register(8, fm.fpioa.UART2_TX, force=True)
fm.register(7, fm.fpioa.UART2_RX, force=True)

uart1 = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096) # to machine
uart2 = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096) # to robot arm

flag = 0x01

dataToMachine = bytearray([0X7B,                      #帧头1
                  0x01,                      #K210数据标志
                  0x00,                      #保留位
                  flag,                      #是否完成
                  0x00,                      #保留位
                  0x00,                      #保留位
                  0x00,                      #保留位
                  0x00,                      #保留位
                  0x00,                      #保留位
                  0X7B^0x01^flag,       #BBC校验位
                  0x7D])


class K210_receive_robot_arm(object):                    # 定义 K210 接收类
    hasGrab      = 0                                 # uint8_t  是否完成一次抓取
    hasSet       = 0                                 # uint8_t  是否完成一次放置
    hasHangUp    = 0                                 # uint8_t   是否完成一次挂上


# 定义 K210 接收类
class K210_receive_machine(object):                    # 定义 K210 接收类
    flag     = 0                                       # uint8_t  是否到达固定位置
    location = 0                                       # uint8_t  当前位置代号


# 定义串口接收数据保存类
class uart_buf_save(object):                         # 定义 uart 接收数据保存类
    uart_buf  = []                                   # 串口缓冲区数组
    data_cnt  = 0                                    # 总数据长度
    state     = 0                                    # 接收状态


# 实例化接收类
From_robot = K210_receive_robot_arm()                                # 实例化 K210_receive() 为 K210
From_machine = K210_receive_machine()


# 接收 STM32 数据
# 实例化类
machine_data  = uart_buf_save()                            # 实例化 uart_buf_prase()
robot_arm_data  = uart_buf_save()


# 串口数据解析 STM32
def Receive_STM32(data_buf):

    confirm = 0x00

    if data_buf[0] == 0x7B and data_buf[10] == 0x7D:

        for i in range(0, 9):
            confirm = confirm ^ data_buf[i]

        #for j in range(0, 11):
            #print("0x%X, " %  machine_data.uart_buf[j])

        if confirm == data_buf[9]:
            if data_buf[1] == 0x00 and data_buf[2] == 0x01:
                From_machine.flag = data_buf[3]
                From_machine.location = data_buf[4]
                machine_data.uart_buf = []
                robot_arm_data.uart_buf = []
                print("OK From_machine")

            elif data_buf[1] == 0x02 and data_buf[2] == 0x01:
                From_robot.hasGrab = data_buf[3]
                From_robot.hasSet = data_buf[4]
                From_robot.hasHangUp = data_buf[5]
                machine_data.uart_buf = []
                robot_arm_data.uart_buf = []
                print("OK From_robot")


cnt1 = 5

# 串口数据接收 STM32
def uart_receive_stm32(buf, who):
    if who == 1:
        print("0x%X," % buf)

        machine_data.uart_buf.append(buf)
        machine_data.data_cnt += 1
        if machine_data.data_cnt == 11:
            #print("%d" % machine_data.data_cnt)
            #for j in range(0, 11):
                #print("0x%X, " %  machine_data.uart_buf[j])
            global cnt1
            cnt1 = cnt1 + 10
            print("-" * cnt1)
            machine_data.data_cnt = 0
            Receive_STM32(machine_data.uart_buf)
    if who == 2:
        print("0x%X," % buf)
        robot_arm_data.uart_buf.append(buf)
        robot_arm_data.data_cnt += 1
        if robot_arm_data.data_cnt == 11:
            global cnt1
            cnt1 = cnt1 + 10
            print("-" * cnt1)
            robot_arm_data.data_cnt = 0
            Receive_STM32(robot_arm_data.uart_buf)



# 串口数据读取 最后调用这个两函数即可
def uart1_stm32_read():
    buf_len = uart1.any()                            # 检查 串口1 是否有内容需要读取 返回等待的字节数量（可能为0）
    for i in range(0,buf_len):                       # 读取 buf_len 个数据
        uart_receive_stm32(uart1.readchar(), 1)         # 接收单个数据 uart1.readchar() 然后将这个数据传递到函数 uart_receive_stm32() 进行 STM32 数据接收

def uart2_stm32_read():
    buf_len = uart2.any()                            # 检查 串口1 是否有内容需要读取 返回等待的字节数量（可能为0）
    for i in range(0,buf_len):                       # 读取 buf_len 个数据
        uart_receive_stm32(uart2.readchar(), 2)         # 接收单个数据 uart1.readchar() 然后将这个数据传递到函数 uart_receive_stm32() 进行 STM32 数据接收


uart1.write(dataToMachine)
#uart2.write(dataToMachine)

while True:
    #uart1.write(dataToMachine)

    uart1_stm32_read()
    uart2_stm32_read()




