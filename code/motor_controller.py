# Untitled - By: miore - 周一 4月 24 2023
from machine import Timer,PWM
import time
import utime
from Maix import GPIO
from fpioa_manager import fm
import _thread
from gyroscope import *


class motor:
    def __init__(self, pwm_pin, in_1, in_2):
        self.PWM = pwm_pin

        self.in_1 = in_1
        self.in_2 = in_2

    def controller(self, speed):
        if speed > 0:
            self.in_1.value(1)
            self.in_2.value(0)
            self.PWM.duty(abs(speed) * 100)

        elif speed == 0:
            self.in_1.value(0)
            self.in_2.value(0)
            self.PWM.duty(0)

        else:
            self.in_1.value(0)
            self.in_2.value(1)
            self.PWM.duty(abs(speed) * 100)

    def extracter(self):
        return(self.PWM.duty, self.in_1.value(), self.in_2.value)


class motor_controller_blocking:
    def __init__(self, pwm_pin_L, in_1_L, in_2_L, pwm_pin_R, in_1_R, in_2_R):
        self.motor_L = motor(pwm_pin_L, in_1_L, in_2_L)
        self.motor_R = motor(pwm_pin_R, in_1_R, in_2_R)

        self.motor_L.controller(0)
        self.motor_R.controller(0)

    def fw(self, t, speed):
        self.motor_L.controller(speed / 2)
        self.motor_R.controller(speed / 2)

        utime.sleep_ms(t)

        self.motor_L.controller(0)
        self.motor_R.controller(0)


    def turning(self, t, direction, speed):
        if direction == 'L':
            self.motor_L.controller(-speed / 2)
            self.motor_R.controller(speed / 2)
        else:
            self.motor_L.controller(speed / 2)
            self.motor_R.controller(-speed / 2)

        utime.sleep_ms(t)

        self.motor_L.controller(0)
        self.motor_R.controller(0)


    def circling(self, t, direction, speed_avg, speed_diff):
        if direction == 'L':
            self.motor_L.controller(speed_avg / 2 - speed_diff / 2)
            self.motor_R.controller(speed_avg / 2 + speed_diff / 2)
        else:
            self.motor_L.controller(speed_avg / 2 + speed_diff / 2)
            self.motor_R.controller(speed_avg / 2 - speed_diff / 2)

        utime.sleep_ms(t)

        self.motor_L.controller(0)
        self.motor_R.controller(0)

    def stop(self):
        self.motor_L.controller(0)
        self.motor_R.controller(0)


class motor_controller_unblocking:
    def __init__(self, pwm_pin_L, in_1_L, in_2_L, pwm_pin_R, in_1_R, in_2_R):
        self.motor_L = motor(pwm_pin_L, in_1_L, in_2_L)
        self.motor_R = motor(pwm_pin_R, in_1_R, in_2_R)

        self.l_speed = 0
        self.r_speed = 0

        self.motor_L.controller(self.l_speed)
        self.motor_R.controller(self.r_speed)


    def run(self, speed_l, speed_r):
        self.motor_L.controller(speed_l)
        self.motor_R.controller(speed_r)


    def fw(self, speed, dif):
        self.l_speed = speed / 2
        self.r_speed = speed / 2
        self.motor_L.controller(self.l_speed + dif)
        self.motor_R.controller(self.r_speed)

    def pause(self, btime):
        self.stop()
        time.sleep(btime)


    def turning(self, direction, speed):
        if direction == 'L':
            self.l_speed = -speed / 2
            self.r_speed = speed / 2
            self.motor_L.controller(self.l_speed)
            self.motor_R.controller(self.r_speed)
        else:
            self.l_speed = speed / 2
            self.r_speed = -speed / 2
            self.motor_L.controller(self.l_speed)
            self.motor_R.controller(self.r_speed)


    def circling(self, direction, speed_avg, speed_diff):
        if direction == 'L':
            self.l_speed = speed_avg / 2 - speed_diff / 2
            self.r_speed = speed_avg / 2 + speed_diff / 2
            self.motor_L.controller(self.l_speed)
            self.motor_R.controller(self.r_speed)
        else:
            self.l_speed = speed_avg / 2 + speed_diff / 2
            self.r_speed = speed_avg / 2 - speed_diff / 2
            self.motor_L.controller(self.l_speed)
            self.motor_R.controller(self.r_speed)


    def stop(self):
        self.l_speed = 0
        self.r_speed = 0
        self.motor_L.controller(self.l_speed)
        self.motor_R.controller(self.r_speed)

    def pause(self, btime):
        self.stop()
        time.sleep(btime)

    def motor_interupt(self, KEY):
        utime.sleep_ms(10)
        print("Right Motor: {}".format(self.r_speed))
        print("Left Motor: {}".format(self.l_speed))

        if KEY.value()==0:
            self.motor_L.controller(self.l_speed)
            self.motor_R.controller(self.r_speed)


def fun(KEY):
    utime.sleep_ms(10)
    if KEY.value()==0:
        mc.stop()


def l_speed_generator(motor):
    while True:
        motor.l_speed = -motor.l_speed
        print("hello {}".format(motor.l_speed))
        time.sleep(1)


def r_speed_generator(motor):
    while True:
        motor.r_speed = -motor.r_speed
        print("hello {}".format(motor.r_speed))
        time.sleep(1)


if __name__ == "__main__":
    flag = 2

    if flag == 0:
        fm.register(16, fm.fpioa.GPIOHS0)
        KEY=GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_UP)

        tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)

        pwm_A = PWM(tim, freq=1e5, duty=0, pin=29)
        fm.register(26, fm.fpioa.GPIO0)
        fm.register(27, fm.fpioa.GPIO1)
        in_1A = GPIO(GPIO.GPIO0, GPIO.OUT)
        in_2A = GPIO(GPIO.GPIO1, GPIO.OUT)

        pwm_B = PWM(tim, freq=1e5, duty=0, pin=31)
        fm.register(28, fm.fpioa.GPIO2)
        fm.register(30, fm.fpioa.GPIO3)
        in_1B = GPIO(GPIO.GPIO2, GPIO.OUT)
        in_2B = GPIO(GPIO.GPIO3, GPIO.OUT)

        global mc
        mc = motor_controller_blocking(pwm_A, in_1A, in_2A, pwm_B, in_1B, in_2B)

        KEY.irq(fun, GPIO.IRQ_FALLING)
        mc.fw(8000, 2)
        #mc.turning(5000, 'L', 2)
        #mc.circling(5000, 'L', 1, 0.2)

    if flag == 1:
        fm.register(16, fm.fpioa.GPIOHS0)
        KEY=GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_UP)

        tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)

        pwm_A = PWM(tim, freq=1e5, duty=0, pin=29)
        fm.register(26, fm.fpioa.GPIO0)
        fm.register(27, fm.fpioa.GPIO1)
        in_1A = GPIO(GPIO.GPIO0, GPIO.OUT)
        in_2A = GPIO(GPIO.GPIO1, GPIO.OUT)

        pwm_B = PWM(tim, freq=1e5, duty=0, pin=31)
        fm.register(28, fm.fpioa.GPIO2)
        fm.register(30, fm.fpioa.GPIO3)
        in_1B = GPIO(GPIO.GPIO2, GPIO.OUT)
        in_2B = GPIO(GPIO.GPIO3, GPIO.OUT)

        global mc
        mc = motor_controller_unblocking(pwm_A, in_1A, in_2A, pwm_B, in_1B, in_2B)
        mc.fw(1)
        KEY.irq(mc.motor_interupt, GPIO.IRQ_FALLING)

        _thread.start_new_thread(l_speed_generator,(mc,))
        _thread.start_new_thread(r_speed_generator,(mc,))

        while True:
            pass

    if flag == 2:
        fm.register(16, fm.fpioa.GPIOHS0)
        KEY=GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_UP)

        tim_A = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
        tim_B = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)

        pwm_A = PWM(tim_A, freq=1e5, duty=0, pin=29)
        fm.register(26, fm.fpioa.GPIO0)
        fm.register(27, fm.fpioa.GPIO1)
        in_1A = GPIO(GPIO.GPIO0, GPIO.OUT)
        in_2A = GPIO(GPIO.GPIO1, GPIO.OUT)

        pwm_B = PWM(tim_B, freq=1e5, duty=0, pin=31)
        fm.register(28, fm.fpioa.GPIO2)
        fm.register(30, fm.fpioa.GPIO3)
        in_1B = GPIO(GPIO.GPIO2, GPIO.OUT)
        in_2B = GPIO(GPIO.GPIO3, GPIO.OUT)

        global mc
        mc = motor_controller_unblocking(pwm_A, in_1A, in_2A, pwm_B, in_1B, in_2B)
        mc.fw(0)

