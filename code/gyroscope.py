'''
陀螺仪角度
使用方法：
1. 实例化一个类，可能参数有baud, RX_pin, TX_pin 默认为9600, 6, 7
2. 直接访问类的实例或调用函数self.get_angle返回全局角度
3. 使用self.set_zero()置零，使用self.switch()开关，注意：关闭状态下获得None
4. 使用self.angle_record_start()开始记录局部角度变化，使用self.angle_change()返回变化量，
   左正右负，范围+-180°。重置局部角度重新使用self.angle_record_start()
'''

from machine import UART,Timer
from fpioa_manager import fm

class gyroscope:
    def __init__(self, baud=9600, RX_pin=6, TX_pin=7):
        self.uart = UART(UART.UART1, 9600, read_buf_len=4096)
        fm.register(RX_pin, fm.fpioa.UART1_RX, force=True)
        fm.register(TX_pin, fm.fpioa.UART1_TX, force=True)
        self.state = True
        self.op_code('\x61')
        self.op_code('\x65')
        # self.d_angle = 0

    def __repr__(self):
        return self.get_angle()

    def op_code(self, code):
        self.uart.write(b'\xff')
        self.uart.write(b'\xaa')
        self.uart.write(code)

    def set_zero(self):
        self.op_code(b'\x52')

    def switch(self):
        self.state = not self.state
        self.op_code(b'\x60')

    def data_read(self):
        data = self.uart.read()
        return data

    def get_angle(self):
        if not self.state:
            print("The gs is off!")
            return None
        while True:
            raw_data = self.data_read()
            if raw_data:
                if raw_data[0] == 85 and raw_data[1] == 81 and len(raw_data)==33:
                    break
                else:
                    continue
            else:
                continue
        return (((raw_data[29]<<8)|raw_data[28])/32768)*180

    def angle_record_start(self):
        self.set_zero()
        self.d_angle = 0
        self.p_angle = self.get_angle()

    def angle_change(self):
        now_angle = self.get_angle()
        self.d_angle = self.d_angle + (now_angle - self.p_angle)
        self.p_angle = now_angle
        if self.d_angle>=180:
            return self.d_angle - 360
        else:
            return self.d_angle



if __name__=="__main__":
    gs = gyroscope()
    gs.set_zero()
    gs.angle_record_start()
    while True:
        print(gs.angle_change())
