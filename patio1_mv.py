# Untitled - By: Cristiano - 周三 5月 31 2023
# from pyb import UART
import sensor, image,lcd
from machine import Timer,PWM
import time
from time import sleep, sleep_us, sleep_ms
import utime
from Maix import GPIO
from fpioa_manager import fm
import _thread
import lcd
from machine import UART
from radar import *
from gyroscope import *
from motor_controller import motor_controller_unblocking
import struct
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
# sensor.set_vflip(True)

img=sensor.snapshot()
width,height=img.width(),img.height()
print(width,height)
ROI_devide=(0,60,20,int(height/2))
clock = time.clock()
section=16
real_width=int(width/section)
half=int(real_width/2)
sleep_ms(20)
door_bias=0
bridge_bias=0

def degree(A, B):
    x = B - A
    d_angle = min(abs(x), 360-abs(x))
    if 0<=x<180 or -360<=x<-180:
        d = 'L'
    else:
        d = 'R'
    return d_angle, d


tim_A = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
tim_B = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
pwm_A = PWM(tim_A, freq=1e5, duty=0, pin=29)    # Y5
fm.register(26, fm.fpioa.GPIO0) # Y7
fm.register(27, fm.fpioa.GPIO1) # Y6
in_1A = GPIO(GPIO.GPIO0, GPIO.OUT)
in_2A = GPIO(GPIO.GPIO1, GPIO.OUT)

pwm_B = PWM(tim_B, freq=1e5, duty=0, pin=31)    # X10
fm.register(28, fm.fpioa.GPIO2) # Y8
fm.register(30, fm.fpioa.GPIO3) # X9
in_1B = GPIO(GPIO.GPIO2, GPIO.OUT)
in_2B = GPIO(GPIO.GPIO3, GPIO.OUT)

mc = motor_controller_unblocking(pwm_A, in_1A, in_2A, pwm_B, in_1B, in_2B)

fm.register(16, fm.fpioa.GPIO4) # Y1
fm.register(18, fm.fpioa.GPIO5) # Y2
fm.register(19, fm.fpioa.GPIO6) # Y3
fm.register(20, fm.fpioa.GPIO7) # Y4

fm.register(10, fm.fpioa.GPIOHS0)
sg90 = GPIO(GPIO.GPIOHS0, GPIO.OUT)

sg90.value(0)
sg90.value(1)
sleep_us(2500)
sg90.value(0)


trig1 = GPIO(GPIO.GPIO4, GPIO.OUT)
echo1 = GPIO(GPIO.GPIO5, GPIO.IN)
trig2 = GPIO(GPIO.GPIO6, GPIO.OUT)
echo2 = GPIO(GPIO.GPIO7, GPIO.IN)

HC1 = HCSR04(trig1, echo1)
HC2 = HCSR04(trig2, echo2)

gs = gyroscope()

def turn(B, delta=0.35):
    btime = 0.5
    A = gs.get_angle()
    if A==B:
        return True
    d_anglei, di = degree(A, B)
    while True:
        d_anglen, dn = degree(gs.get_angle(), B)
        mc.circling(di, 0, 0.5*d_anglen/d_anglei+delta)
        if dn!=di:
            break
    mc.pause(btime)

def radar_move(HC, com, dis, dif=0.0248):
    print(dif)
    arr_size_1 = 10
    div_time_1 = 0.02
    dis_to_fence_1 = 50
    btime = 0.5
    if HC=='F':
        gd = lambda: HC2.getDistance()
    else:
        gd = lambda: HC1.getDistance()
    print('move begin')
    mc.fw(0.8, dif)
    count = 0
    if com=='>':
        dis_list = [0]*arr_size_1
        while True:
            dis_list[count%arr_size_1] = gd()
            print(dis_list[count%arr_size_1])
            sleep(div_time_1)
            count = count+1
            dis_list_copy = dis_list.copy()
            dis_list_copy.sort()
            if dis_list[arr_size_1//2]>dis:
                break
    else:
        dis_list = [1000]*arr_size_1
        while True:
            dis_list[count%arr_size_1] = gd()
            print(dis_list[count%arr_size_1])
            sleep(div_time_1)
            count = count+1
            dis_list_copy = dis_list.copy()
            dis_list_copy.sort()
            if dis_list[arr_size_1//2]<dis:
                break
    if HC=='R':
        sleep(1.5)
    mc.pause(btime)
    print('move end')

# uart = UART(3, 115200)
"""
def send_data_packet(line_bias,door_bias,bridge_bias,status):
    data =struct.pack("<bbhhhh",              #格式为四个字符俩个短整型(2字节)
                   0x29,                       #帧头1
                   0x12,                       #帧头2
                   int(line_bias), # up sample by 4    #数据1,低位在前
                   int(door_bias), # up sample by 4    #数据2.
                   int(bridge_bias), # up sample by 4    #数据2.
                   int(status) # up sample by 4    #数据2.
                   )
    uart.write(data)
"""
#求目标颜色中值
def find_tar(meanvalue,section):
    allsum=sum(meanvalue)

    sum1=0
    for i in range(section):
        if sum1<=allsum/2:
            sum1+=meanvalue[i]
        elif i==section-1:
            return i
        else:
            return i-1

def step_4():
    radar_move('F','<',40, dif=0.0508)
    turn(270)

def step_5():
    # self.radar_move('R','>',100, 0.0388)
    radar_move('F','<',40, 0.0508)
    turn(0)
    radar_move("R", '<', 20, 0.0518)

#洗数据
#可以加强归一化和拉大相邻差，但是会被影子影响严重
def clean(data,threshold_shit=None):
    if threshold_shit is None:
        #data[0]=0
        #data[-1]=0
        #摄像头左右两侧图像变形且像素变小
        avg=sum(data)/(len(data))
        if max(data)<190:
        #下午太阳后续白线像素和，下于就是寻黑线，反之白线
            for i in range(len(data)):
                if data[i]<1:
                    data[i]=0
            return data
        else :
            for i in range(len(data)):
                if data[i]<avg-5:
                    data[i]=0
            return data

    else :
    #预先设定阈值  不好用
        for i in range(len(data)):
            if data[i]>threshold_shit:
                data[i]=0
        return data

if __name__ == "__main__":
    sleep(3)
    gs.set_zero()
    while(True):
        data=[]
        clock.tick()
        img = sensor.snapshot()
        img = img.rotation_corr(z_rotation=-90)
        kernel_size = 1
        kernel = [-1, -1, -1,\
        -1, +8, -1,\
        -1, -1, -1]
        # img.find_edges(image.EDGE_CANNY,threshold=(80,160))
        img = img.morph(kernel_size, kernel)
        for i in range(section):
            ROI_cal=(i*real_width,ROI_devide[1],ROI_devide[2],ROI_devide[3])
            #print(ROI_cal)
            #img.draw_rectangle(ROI_cal)
            statistics=img.get_statistics(roi=ROI_cal)
            data.append(statistics[0])
            img.draw_rectangle(ROI_cal)

        #print('avg',sum(data)/(len(data)-2))
        # print(data,clean(data))
        #可以选择预设参数，这需要拿着车手动滚一圈
        #index=find_tar(clean(data,threshold_shit=130),section)
        index=find_tar(data,section)
        if index==None:
            index=section/2

        img.draw_cross(int(real_width*(index+1)),120,color=255,thickness=10)
        direct=index+1-section/2
        print(direct)
        expected = 0
        actual = direct
        error_n = 0
        error_b = 0
        extra_drop = 0.1
        expected = 0
        kp = 0.06
        ki = 0.0001
        kd = 0.06
        sum_error = 0
        d_error = 0
        for i in range(10):
            err =  actual - expected
            error_b = error_n
            error_n = err
            d_error = error_n - error_b
            sum_error += err
            dif = kp * err + ki * sum_error + kd * d_error
            mc.run(0.35+dif, 0.35-dif)
            print("Speed: L:{}, R:{}".format(0.35+dif, 0.35-dif))
            sleep_ms(7)
            actual += dif - extra_drop
            err = expected - actual
        dis = HC2.getDistance()
        dis2 = HC1.getDistance()
        if dis <= 50 and dis2 > 500:
            step_4()
            step_5()
            break
        # output=direct*10
        # dif = output / 80
        # mc.run(0.4 + dif, 0.4 - dif)
        #print('index=',index,data,(index-section/2))
        # send_data_packet(output,door_bias,bridge_bias,status=1)
        # print(clock.fps())
        # sleep_ms(15)

