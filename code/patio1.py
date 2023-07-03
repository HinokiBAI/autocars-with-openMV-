import sensor, image,lcd
from machine import Timer,PWM
import time
from time import sleep, sleep_us, sleep_ms
import utime
from Maix import GPIO
from fpioa_manager import fm
import _thread
import sensor, image,lcd
from machine import UART
from fpioa_manager import fm
from radar import *
from gyroscope import *
from machine import UART
from motor_controller import motor_controller_unblocking
from fpioa_manager import fm


# (1, 89, -9, -1, 7, 28), (0, 100, -5, 0, 0, 12)
# 26, 123
bin_thre = ((26, 123))
# white_thre = ((160, 255))
white_thre = ((28, 100, -11, -2, -1, 23))              #白色
roi1            = [0,150,320,16]       #巡线敏感区
expected   = 160                  #巡线位置期望
err             = 0                    #本次误差
old_err         = 0                    #上次误差
Speed           = 0.5
Speed_left      = 0
Speed_right     = 0

def degree(A, B):
    x = B - A
    d_angle = min(abs(x), 360-abs(x))
    if 0<=x<180 or -360<=x<-180:
        d = 'L'
    else:
        d = 'R'
    return d_angle, d

class patio1:
    def __init__(self, mc, gs, HC1, HC2, sg90):
        self.mc = mc
        self.gs = gs
        self.HC1 = HC1
        self.HC2 = HC2
        self.sg90 = sg90

        self.gs.set_zero()

        # constant
        self.btime = 0.5
        # step4
        self.arr_size_1 = 10
        self.div_time_1 = 0.02
        self.dis_to_fence_1 = 50
        # step5
        self.arr_size_2 = 10
        self.div_time_2 = 0.02
        self.dis_to_fence_2 = 30

    def turn(self, B):
        A = self.gs.get_angle()
        d_anglei, di = degree(A, B)
        while True:
            d_anglen, dn = degree(self.gs.get_angle(), B)
            self.mc.circling(di, 0, 0.5*d_anglen/d_anglei+0.3)
            if dn!=di:
                break
        self.mc.pause(self.btime)

    def radar_move(self, HC, com, dis, dif=0.0155):
        if HC=='F':
            gd = lambda: self.HC2.getDistance()
        else:
            gd = lambda: self.HC1.getDistance()
        print('move begin')
        self.mc.fw(1, dif)
        count = 0
        if com=='>':
            dis_list = [0]*self.arr_size_1
            while True:
                print(dis_list)
                dis_list[count%self.arr_size_1] = gd()
                sleep(self.div_time_1)
                count = count+1
                dis_list_copy = dis_list.copy()
                dis_list_copy.sort()
                if dis_list[self.arr_size_1//2]>dis:
                    break
        else:
            dis_list = [1000]*self.arr_size_1
            while True:
                print(dis_list)
                dis_list[count%self.arr_size_1] = gd()
                sleep(self.div_time_1)
                count = count+1
                dis_list_copy = dis_list.copy()
                dis_list_copy.sort()
                if dis_list[self.arr_size_1//2]<dis:
                    break

        self.mc.pause(self.btime)
        print('move end')

    def color_rec(self, img, rec_thres, roi):
        # img = img.binary(bin_thres)
        stat = img.find_blobs([rec_thres], roi=roi, area_threshold=200, merge=True)
        Speed_left = 0
        Speed_right = 0
        dif = 0
        if stat:
            for blobs in stat:
                tmp=img.draw_rectangle(blobs[0:4])
                tmp=img.draw_cross(blobs[5], blobs[6])
                lcd.display(tmp)
                #PID
                actual=blobs[5]
                print(actual)
                error_n = 0
                error_b = 0
                extra_drop = 0.1
                expected = 160
                kp = -0.0001
                ki = 0
                kd = 0.0001
                sum_error = 0
                d_error = 0
                old_err = 0
                actualValue=blobs[5]
                for i in range(10):
                    err=actual-expected
                    dif = (kp*err+kd*(err-old_err))
                    Speed_left = 0.4 + dif
                    Speed_right = 0.4
                    old_err= err
                print("Speed_left,Speed_right")
                print(Speed_left,Speed_right)
                self.mc.fw(0.8, dif)
                sleep(0.1)


    def bridge(self, dis):

        if dis < 30:
            self.radar_move('R', '<', 20, 270)
            self.mc.fw(0.8, 0.0384)
            sleep(5)
            self.radar_move('R', '<', 20, 0)
            self.mc.fw(0.8, 0.0384)
            sleep(5)


if __name__ == "__main__":
    lcd.init()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_vflip(True)
    sensor.set_framesize(sensor.QVGA) # 320x240
    sensor.skip_frames(time = 3000 )#跳过3000张图片
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.run(1)

    print('begin')

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

    model = patio1(mc, gs, HC1, HC2, sg90)
    sleep(3)

    while True:
        img=sensor.snapshot()
        img = img.rotation_corr(z_rotation=-90)
        # img = img.binary((bin_thre))
        # lcd.display(img)
        # print(type(img))
        # print(type(bin_thre))
        model.color_rec(img, white_thre, roi1)
        dis = HC1.getDistance()
        model.bridge(dis)

    # mc.fw(tim, Speed)
