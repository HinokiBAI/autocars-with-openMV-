from time import sleep_us, ticks_us
import time
from Maix import GPIO
from fpioa_manager import fm


class HCSR04():
    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo

    def getDistance(self):
        distance = 0
        self.trig.value(1)
        sleep_us(20)
        self.trig.value(0)
        while self.echo.value() == 0:
            pass
        if self.echo.value() == 1:
            ts = ticks_us()  # 开始时间
            while self.echo.value() == 1:
                pass
            te = ticks_us()  # 时间结束
            tc = te - ts
            distance = tc * 0.017  # 距离计算（单位： cm）
        return distance


if __name__ == "__main__":
    flag = 1

    if flag == 0:
        fm.register(18, fm.fpioa.GPIO0)
        fm.register(16, fm.fpioa.GPIO1)
        fm.register(19, fm.fpioa.GPIO2)
        trig = GPIO(GPIO.GPIO0, GPIO.OUT)
        echo = GPIO(GPIO.GPIO1, GPIO.IN)
        vcc = GPIO(GPIO.GPIO2, GPIO.OUT)
        vcc.value(1)
        HC = HCSR04(trig, echo)

        i = 0

        for i in range(20):
            distance = HC.getDistance()
            print(str(distance) + ' cm')
            time.sleep(2)


    if flag == 1:
        fm.register(16, fm.fpioa.GPIO0)
        fm.register(18, fm.fpioa.GPIO1)
        fm.register(19, fm.fpioa.GPIO2)
        fm.register(20, fm.fpioa.GPIO3)


        trig1 = GPIO(GPIO.GPIO0, GPIO.OUT)
        echo1 = GPIO(GPIO.GPIO1, GPIO.IN)
        trig2 = GPIO(GPIO.GPIO2, GPIO.OUT)
        echo2 = GPIO(GPIO.GPIO3, GPIO.IN)

        HC1 = HCSR04(trig1, echo1)
        HC2 = HCSR04(trig2, echo2)

        for i in range(20):
            distance1 = HC1.getDistance()
            # distance2 = HC2.getDistance()
            print(str(distance1) + ' cm')
            # print(str(distance2) + ' cm')
            time.sleep(2)

