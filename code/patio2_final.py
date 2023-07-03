# Untitled - By: 10031 - 周六 5月 27 2023

from motor_controller import *
from radar import *
from hc12_module import *
from time import sleep_us, sleep
from gyroscope import *

import sensor, lcd

from arrow_rec import ArrowRec

def degree(A, B):
    x = B - A
    d_angle = min(abs(x), 360-abs(x))
    if 0<=x<180 or -360<=x<-180:
        d = 'L'
    else:
        d = 'R'
    return d_angle, d


class patio2:
    def __init__(self, mc, gs, HC1, HC2, sg90, hc12):
        self.mc = mc
        self.gs = gs
        self.HC1 = HC1
        self.HC2 = HC2
        self.sg90 = sg90
        self.hc12 = hc12
        self.gs.set_zero()

        # constant
        self.btime = 0.5

        # step1
        self.step_1_time = 7.3

        # step2

        # step3
        self.step_3_time = [12, 18.2, 13]
        # step4
        self.arr_size_1 = 10
        self.div_time_1 = 0.02
        self.dis_to_fence_1 = 50
        # step5
        self.arr_size_2 = 10
        self.div_time_2 = 0.02
        self.dis_to_fence_2 = 30
        # step6
        self.arr_size_1 = 10
        self.div_time_1 = 0.02
        self.dis_to_fence_1 = 50
        # step7
        self.dis_to_fence_3 = 30
        self.dis_to_bin = 40
        self.step_7_time = 8
        # step8
        self.step_8_time = 4
        self.dis_to_tree = 80

    def turn(self, B, delta=0.35):
        A = self.gs.get_angle()
        if A==B:
            return True
        d_anglei, di = degree(A, B)

        while True:
            d_anglen, dn = degree(self.gs.get_angle(), B)
            self.mc.circling(di, 0, 0.5*d_anglen/d_anglei+delta)
            if dn!=di:
                break
        self.mc.pause(self.btime)


    def radar_move(self, HC, com, dis, dif=0.0248):
        print(dif)
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
            sleep(1.5)
        self.mc.pause(self.btime)
        print('move end')




    def step_1(self):
        print('go')
        error_n = 0
        error_b = 0
        extra_drop = 0.1
        expected = 0
        kp = 0.01
        ki = 0.0001
        kd = 0.001
        sum_error = 0
        d_error = 0
        for i in range(self.step_1_time):
            actual = self.gs.get_angle()
            if actual <= 180:
                actual = actual
            else:
                actual = actual - 360
            for j in range(20):
                err =  actual - expected
                error_b = error_n
                error_n = err
                d_error = error_n - error_b
                sum_error += err
                dif = kp * err + ki * sum_error + kd * d_error
                print("speed: L{}, R:{}".format(0.4+dif, 0.4))
                self.mc.fw(0.8, dif)
                sleep(1/20)
                actual += dif - extra_drop
                err = expected - actual
        # self.mc.fw(0.8, 0.0368)
        # print("Step_1-FW-L:{}, R:{}".format(self.mc.l_speed, self.mc.r_speed))
        # sleep(self.step_1_time)
        self.mc.stop()
        print("Step_1-STOP-L:{}, R:{}".format(self.mc.l_speed, self.mc.r_speed))


    def step_2(self):
        print("wait to be input")
        self.hc12.set_channel(021)
        while True:
            message = self.hc12.read()
            print(message)
            if len(message)!=9:
                continue
            if message[0:2]=='55':
                if '101' in message:
                    direction = 'R'
                elif '102' in message:
                    direction = 'U'
                elif '103' in message:
                    direction = 'L'
                else:
                    continue
                mytime = message[-4:]
                return direction, mytime
            else:
                continue
        """
        print("?")
        lcd.init()
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_vflip(True)
        sensor.run(1)
        sensor.skip_frames(30)
        img = sensor.snapshot()
        img = img.rotation_corr(z_rotation=-90)
        img.save(r'/sd/bty.jpg')
        lcd.display(img)
        a = ArrowRec(img,need_threshold=(40, 100))
        b = a.rec_arrow()
        print(b)
        sensor.run(0)
        sensor.shutdown(False)

        try:
            img.draw_string(0, 0, b, color = (255, 255,255), scale = 3,mono_space = False)
        except:
            pass
        lcd.display(a.draw_blob())

        return b
        """


    def step_3(self, direction):
        print("Step: 3")
        if direction == 'R':
            self.turn(35)
            self.mc.fw(0.8,0.0408)
            sleep(self.step_3_time[0])

        elif direction == 'U':
            self.turn(90)
            self.mc.fw(0.8,0.0408)
            sleep(self.step_3_time[1])

        elif direction == 'L':
            self.turn(135)
            self.mc.fw(0.8,0.0408)
            sleep(self.step_3_time[2])


    def step_4(self):
        self.turn(0)
        self.radar_move('F','<',40, dif=0.0508)
        self.turn(90)
        # self.turn(270)

    def step_5(self):
        self.radar_move('R','>',100, 0.0508)
        # self.radar_move('F','<',40, 0.0508)
        # self.turn(0)
        # self.radar_move("R", '<', 20, 0.0508)

    def step_6(self):
        self.turn(5)
        # self.mc.fw(0.8,0.0348)
        # sleep(15)
        # self.turn(0)
        self.radar_move('F','<',40, 0.0548)
        self.turn(90)

    def step_7(self):
        #self.radar_move("F", "<", self.dis_to_bin, 0.04)
        self.mc.fw(0.8, 0.0448)
        sleep(self.step_7_time)
        sg90.value(0)
        sg90.value(1)
        sleep_us(2500)
        sg90.value(0)
        self.mc.pause(self.btime)


    def step_8(self):
        self.turn(160, 0.40)
        self.radar_move("F", "<", self.dis_to_tree, 0.036)
        self.mc.fw(0.8, 0,0368)
        sleep(self.step_8_time)
        self.mc.pause(self.btime)

    def step_9(self, mytime):
        while True:
            hour = int(mytime[0:2])
            minute = int(mytime[2:4])
            string = '2023/6/3 %d:%d'%(hour,minute)
            hc12.command('DEFAULT')
            hc12.send(string)


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
    hc12 = HC12(RX_pin=15, TX_pin=8, SET_pin=33)

    # run
    print("ready")
    p2 = patio2(mc, gs, HC1, HC2, sg90, hc12)
    sleep(3)


    gs.set_zero()
    # p2.step_1()
    # d,mytime = p2.step_2()
    # print(d, mytime)
    # p2.step_3('R')
    # p2.step_4()
    # p2.step_5()
    p2.step_4()
    p2.step_5()
    p2.step_6()
    p2.step_7()
    p2.step_8()
    # sleep(10)
    p2.step_9(mytime)



    # a = p2.step_4()
    # print(a)



