#!/usr/bin/env python3
# encoding: utf-8
import time
import ctypes
import serial
import RPi.GPIO as GPIO
#幻尔科技总线舵机通信#

LOBOT_SERVO_FRAME_HEADER         = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = 7
LOBOT_SERVO_MOVE_TIME_WAIT_READ  = 8
LOBOT_SERVO_MOVE_START           = 11
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_ANGLE_OFFSET_READ    = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    = 20
LOBOT_SERVO_ANGLE_LIMIT_READ     = 21
LOBOT_SERVO_VIN_LIMIT_WRITE      = 22
LOBOT_SERVO_VIN_LIMIT_READ       = 23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ  = 25
LOBOT_SERVO_TEMP_READ            = 26
LOBOT_SERVO_VIN_READ             = 27
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ   = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = 32
LOBOT_SERVO_LED_CTRL_WRITE       = 33
LOBOT_SERVO_LED_CTRL_READ        = 34
LOBOT_SERVO_LED_ERROR_WRITE      = 35
LOBOT_SERVO_LED_ERROR_READ       = 36

serialHandle = serial.Serial("/dev/ttyS0", 115200)  # 初始化串口， 波特率为115200

rx_pin = 7
tx_pin = 13

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

def portInit():  # 配置用到的IO口
    GPIO.setup(rx_pin, GPIO.OUT)  # 配置RX_CON 即 GPIO17 为输出
    GPIO.output(rx_pin, 0)
    GPIO.setup(tx_pin, GPIO.OUT)  # 配置TX_CON 即 GPIO27 为输出
    GPIO.output(tx_pin, 1)

portInit()

def portWrite():
    # BOTH HIGH: TX Enabled, RX Disabled
    GPIO.output(tx_pin, 1)
    GPIO.output(rx_pin, 1)

def portRead():
    # BOTH LOW: TX Disabled, RX Enabled
    GPIO.output(rx_pin, 0)
    GPIO.output(tx_pin, 0)

def portRest():
    time.sleep(0.1)
    serialHandle.close()
    GPIO.output(rx_pin, 1)
    GPIO.output(tx_pin, 1)
    serialHandle.open()
    time.sleep(0.1)

def checksum(buf):
    # 计算校验和
    sum = 0x00
    for b in buf:  # 求和
        sum += b
    sum = sum - 0x55 - 0x55  # 去掉命令开头的两个 0x55
    sum = ~sum  # 取反
    return sum & 0xff

def serial_serro_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    '''
    写指令
    :param id:
    :param w_cmd:
    :param dat1:
    :param dat2:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(id)
    # 指令长度
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    buf.append(w_cmd)  # 指令
    # 写数据
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # 偏差
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # 分低8位 高8位 放入缓存
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # 分低8位 高8位 放入缓存
    # 校验和
    buf.append(checksum(buf))
    # for i in buf:
    #     print('%x' %i)
    serialHandle.write(buf)  # 发送

def serial_servo_read_cmd(id=None, r_cmd=None):
    portWrite()
    serialHandle.reset_input_buffer() # Clear old garbage before we send
    
    buf = bytearray(b'\x55\x55')
    buf.append(id)
    buf.append(3)
    buf.append(r_cmd)
    buf.append(checksum(buf))
    
    serialHandle.write(buf)
    serialHandle.flush() # Wait for OS to push data to the wire
    
    # Wait for bits to physically leave the wire
    tx_time = len(buf) * 0.000087
    time.sleep(tx_time + 0.0001)
    
    portRead() # Switch to read mode immediately
    # CRITICAL: Do not clear the buffer here!

def serial_servo_get_rmsg(cmd):
    # We are already in portRead() from the end of serial_servo_read_cmd
    time.sleep(0.005)  # Wait for servo to finish replying
    
    count = serialHandle.inWaiting()
    if count == 0:
        return None
        
    recv_data = serialHandle.read(count)
    
    # Search through the buffer to find the correct reply, skipping the echo
    for i in range(len(recv_data) - 4):
        # 1. Match Header (55 55) and Command
        if recv_data[i] == 0x55 and recv_data[i+1] == 0x55 and recv_data[i+4] == cmd:
            dat_len = recv_data[i+3]
            
            # 2. IGNORE THE ECHO: 
            # Sent commands usually have length 3 or 4. Read replies are 4, 5, or 7.
            # If we asked for POS_READ (28), the reply length MUST be 5.
            if cmd == LOBOT_SERVO_POS_READ and dat_len != 5:
                continue
                
            # 3. Extract the data safely
            if i + dat_len + 2 <= len(recv_data):
                if dat_len == 4:
                    return recv_data[i+5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[i+5] | (0xff00 & (recv_data[i+6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[i+5] | (0xff00 & (recv_data[i+6] << 8)))
                    pos2 = 0xffff & (recv_data[i+7] | (0xff00 & (recv_data[i+8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
    return None