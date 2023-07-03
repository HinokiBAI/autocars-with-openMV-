from motor_controller import *
from radar import *

from time import sleep_us, sleep
from gyroscope import *

import sensor, lcd

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
        self.btime = 0.2

        self.gs.set_zero()

        self.step_1_time = 17

    def turn(self, B, delta=0.35):
        A = self.gs.get_angle()
        if A==B:
            return True
        d_anglei, di = degree(A, B)
        while True:
            d_anglen, dn = degree(self.gs.get_angle(), B)
            if d_anglei < 20:
                self.mc.circling(di, 0, 0.35)
            else:
                self.mc.circling(di, 0, 0.5*d_anglen/d_anglei+delta)

            if dn!=di:
                break

    def turn_curve(self, B, delta=0.35):
        A = self.gs.get_angle()
        if A==B:
            return True
        d_anglei, di = degree(A, B)
        while True:
            d_anglen, dn = degree(self.gs.get_angle(), B)
            self.mc.circling(di, 0.5, 0.2)
            if dn!=di:
                break
        self.mc.pause(self.btime)


    def go_straight(self, B, speed, t):
        start_time = time.time()
        A = self.gs.get_angle()
        d_anglei, di = degree(A, B)
        while True:
            now_time = time.time()
            print(now_time - start_time)
            d_anglen, dn = degree(self.gs.get_angle(), B)
            if (now_time - start_time) > t:
                break
            if d_anglen > 6:
                self.turn(0)
            else:
                self.mc.circling(dn, speed, 1 * d_anglen / 8)
            sleep(0.1)

        self.mc.pause(self.btime)


    def radar_move(self, HC, com, dis, dif=0.0248):
        if HC=='F':
            gd = lambda: self.HC2.getDistance()
        else:
            gd = lambda: self.HC1.getDistance()
        print('move begin')
        self.mc.fw(0.8, dif)
        count = 0
        if com=='>':
            dis_list = [0]*self.arr_size_1
            while True:
                dis_list[count%self.arr_size_1] = gd()
                print(dis_list[count%self.arr_size_1])
                sleep(self.div_time_1)
                count = count+1
                dis_list_copy = dis_list.copy()
                dis_list_copy.sort()
                if dis_list[self.arr_size_1//2]>dis:
                    break
        else:
            dis_list = [1000]*self.arr_size_1
            while True:
                dis_list[count%self.arr_size_1] = gd()
                print(dis_list[count%self.arr_size_1])
                sleep(self.div_time_1)
                count = count+1
                dis_list_copy = dis_list.copy()
                dis_list_copy.sort()
                if dis_list[self.arr_size_1//2]<dis:
                    break
        if HC=='R':
            sleep(1)
        self.mc.pause(self.btime)
        print('move end')

    def step_1(self):
        self.radar_move('R','<',60, 0.0365)
        self.fw(0.8, 0.0365)
        sleep(7)
        self.turn_curve(180)

    def step_2(self):
        print('step 2')
        self.radar_move('F','<',60)
        self.turn_curve(0)




if __name__ == "__main__":
    # Initialization
    print(1)
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
    sleep_us(1500)
    sg90.value(0)


    trig1 = GPIO(GPIO.GPIO4, GPIO.OUT)
    echo1 = GPIO(GPIO.GPIO5, GPIO.IN)
    trig2 = GPIO(GPIO.GPIO6, GPIO.OUT)
    echo2 = GPIO(GPIO.GPIO7, GPIO.IN)

    HC1 = HCSR04(trig1, echo1)
    HC2 = HCSR04(trig2, echo2)

    gs = gyroscope()

    # run
    print("ready")
    p1 = patio1(mc, gs, HC1, HC2, sg90)
    sleep(3)


    gs.set_zero()
    # p1.step_1()
    # p1.step_2()
    p1.go_straight(0, 0.8, 100000)




    # a = p2.step_4()
    # print(a)
