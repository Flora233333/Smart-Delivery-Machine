# uart - By: charles - 周三 11月 2 2022

import sensor, image, time, math, utime, lcd                # 导入感光元件模块 sensor 机器视觉模块 image 跟踪运行时间模块 time 数学函数模块 math
import machine                                              # 导入模块 machine
from machine import UART                                    # 从 machine 模块中导入 双向串行通信模块 UART
from fpioa_manager import fm                                # 从 fpioa_manager 模块中导入 引脚注册模块 fm
from Maix import GPIO                                       # 从 Maix 模块中导入 引脚模块 GPIO


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


# 管脚定义
fm.register(20, fm.fpioa.UART1_TX, force=True)
fm.register(21, fm.fpioa.UART1_RX, force=True)

fm.register(23, fm.fpioa.UART2_TX, force=True)
fm.register(22, fm.fpioa.UART2_RX, force=True)


# 实例化:
# 串口实例化
uart1 = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096) # to machine
uart2 = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096) # to robot arm
# 实例化接收类
From_robot = K210_receive_robot_arm()                      # 实例化 K210_receive() 为 K210
From_machine = K210_receive_machine()
# 实例化接收STM32数据类
machine_data  = uart_buf_save()                            # 实例化 uart_buf_prase()
robot_arm_data  = uart_buf_save()


# 用于串口的全局变量:
complete_flag = 0x00
grab_flag = 0x00
set_flag = 0x00
hangup_flag = 0x00
waitGrab_flag = 0x00

QR_code = [0x00, 0x00, 0x00]

next_location = 0x00


# 串口函数:
def data_pack(mode):
    if mode == 1:
        dataToMachine = [0X7B,                      #帧头1
                         0x01,                      #K210数据标志
                         0x00,                      #接收地址
                         complete_flag,             #是否完成
                         0x00,                      #保留位
                         QR_code[0],                #二维码数据
                         QR_code[1],                #二维码数据
                         QR_code[2],                #二维码数据
                         next_location,             #位置代号3中下一次应该前往的颜色
                         0X7B^0x01^complete_flag^QR_code[0]^QR_code[1]^QR_code[2]^next_location,#BBC校验位
                         0x7D]
        return dataToMachine

    elif mode == 2:
        dataToRobot = [0X7B,                      #帧头1
                       0x01,                      #K210数据标志
                       0x02,                      #接收地址
                       grab_flag,                 #进行一次抓取
                       set_flag,                  #进行一次放置
                       hangup_flag,               #进行一次挂上
                       0x00,                      #保留位
                       0x00,                      #保留位
                       0x00,                      #保留位
                       0X7B^0x01^0x02^grab_flag^set_flag^hangup_flag,#BBC校验位
                       0x7D]
        return dataToRobot

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


#cnt1 = 5
flag__1 = 0
flag__2 = 0
# 串口数据接收 STM32
def uart_receive_stm32(buf, who):
    global flag__1
    global flag__2

    if who == 1:
        if buf == 0x7B:
            flag__1 = 1
        if flag__1 == 0:
            return
        if buf == 0x7D and machine_data.data_cnt < 10:
            flag__1 = 0
            machine_data.data_cnt = 0
            machine_data.uart_buf = []
            return
        #print("0x%X," % buf)

        machine_data.uart_buf.append(buf)
        machine_data.data_cnt += 1

        if machine_data.data_cnt == 11:

            flag__1 = 0

            #for j in range(0, 11):
                #print("0x%X, " %  machine_data.uart_buf[j])
            #global cnt1
            #cnt1 = cnt1 + 10
            #print("-" * cnt1)

            machine_data.data_cnt = 0
            Receive_STM32(machine_data.uart_buf)

    if who == 2:
        if buf == 0x7B:
            flag__2 = 1
        if flag__2 == 0:
            return
        if buf == 0x7D and robot_arm_data.data_cnt < 10:
            flag__2 = 0
            robot_arm_data.data_cnt = 0
            robot_arm_data.uart_buf = []
            return
        #print("0x%X," % buf)

        robot_arm_data.uart_buf.append(buf)
        robot_arm_data.data_cnt += 1

        if robot_arm_data.data_cnt == 11:

            flag__2 = 0

            #for j in range(0, 11):
                #print("0x%X, " %  robot_arm_data.uart_buf[j])
            #global cnt1
            #cnt1 = cnt1 + 10
            #print("-" * cnt1)

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


# 用于识别的全局变量
color_threshold = [
                    (20, 0, 280, 70),
                    (20, 80, 280, 70),
                    (3, 65, 1, 34, -46, -15)  # 蓝色
                  ]
ROIS = (140, 20, 130, 170)

cnt = 0

identity_area = 7000

def wait_robot_arm(mode):
    global waitGrab_flag
    global cnt

    if mode == 1:
        if From_robot.hasGrab == 1:
            waitGrab_flag = 0
            From_robot.hasGrab = 0
            cnt += 1 #检测下一个颜色阈值

    elif mode == 2:
        if From_robot.hasSet == 1:
            pass
    elif mode == 3:
        if From_robot.hasHangUp == 1:
            pass


# 识别函数:
def find_bottle(img, roi):
    global grab_flag
    global waitGrab_flag
    global complete_flag
    global cnt

    max_Pixels = -1
    max_ID = -1
    if waitGrab_flag == 0:
        if img:
            blobs = img.find_blobs([color_threshold[2]], roi=roi, area_threshold=100, merge=True, margin=5)# color_threshold[QR_code[cnt]])
            if blobs:
                for n in range(len(blobs)):
                    if blobs[n].pixels() > max_Pixels:
                        max_Pixels = blobs[n].pixels()
                        max_ID = n
                    if(blobs[max_ID].area() >= identity_area):
                        grab_flag = QR_code[cnt]

                        data = data_pack(2)

                        uart2.write(bytearray(data))

                        grab_flag = 0x00

                        waitGrab_flag = 1

                        img.draw_rectangle(blobs[max_ID].rect())

                        img.draw_cross(blobs[max_ID].cx(),
                                       blobs[max_ID].cy())
            else:
                return 0
        else:
            return 0
    else:
        wait_robot_arm(1)
        if cnt >= 3: #抓完了
            From_machine.flag = 0
            From_machine.location = 0
            complete_flag = 0x01

            data = data_pack(1)

            uart1.write(bytearray(data))

            complete_flag = 0x00


def identify_QR_code():
    global QR_code
    global complete_flag

    res = img.find_qrcodes()

    if len(res) > 0:
       code = res[0].payload()
       QR_code[0] = int(code[0])
       QR_code[1] = int(code[1])
       QR_code[2] = int(code[2])

       From_machine.flag = 0
       From_machine.location = 0
       complete_flag = 0x02

       data = data_pack(1)

       uart1.write(bytearray(data))

       complete_flag = 0x00

       #for j in range(3):
            #print("%d" % QR_code[j], end="")

# 初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.skip_frames(30)

lcd.init()
lcd.rotation(2)



#data = data_pack(1)
#uart1.write(bytearray(data))
#uart2.write(bytearray(data))

while True:
    img = sensor.snapshot()

    img.draw_rectangle(ROIS)

    if From_machine.flag == 1 and From_machine.location == 1:
        identify_QR_code()

    if From_machine.flag == 1 and From_machine.location == 3:
        find_bottle(img, ROIS)

    lcd.display(img)

    uart1_stm32_read()

    uart2_stm32_read()




