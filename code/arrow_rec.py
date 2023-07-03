import sensor, image, time ,lcd
from Maix import GPIO
from fpioa_manager import fm

class ArrowRec:
    def __init__(self,img_ori,need_threshold=(10, 83)):
        # (19, 65, -1, 8, -12, 0)
        self.need_threshold = need_threshold
        self.img_ori = img_ori
        # self.img_grey = self.grayscale()
        self.blobs = self.get_blob()
        self.img_cut = self.cutting()

    def get_blob(self):
        blobs = self.img_ori.find_blobs([self.need_threshold],pixels_threshold=500)
        compare = 0
        count = -1
        for i in range(len(blobs)):
            if blobs[i][4]>compare:
                count = i
                compare = blobs[i][4]
        return blobs[count]

    def draw_blob(self):
        tmp = self.img_ori.copy()
        if self.blobs:
            print(self.blobs)
            tmp.draw_rectangle(self.blobs[0:4])
            tmp.draw_cross(self.blobs[5], self.blobs[6])
            return tmp
        else:
            print("no blobs find!")
            return False

    def cutting(self):
        if self.blobs:
            return self.img_ori.copy(roi = self.blobs[0:4])
        else:
            print("no blobs find!")
            return False

    def rec_arrow(self):
        if self.blobs:
            h = self.blobs[3]
            w = self.blobs[2]
            if h-w>40:
                return "U"
            else:
                half_w = int(w/2)
                left_half = self.img_cut.copy(roi = (0, 0, half_w, h))
                right_half = self.img_cut.copy(roi = (half_w, 0, half_w, h))
                l_value = left_half.find_blobs([self.need_threshold])[0][4]
                r_value = right_half.find_blobs([self.need_threshold])[0][4]
                if l_value>r_value:
                    return "L"
                else:
                    return "R"
        else:
            print("no blobs find!")
            return False

if __name__=='__main__':
    #sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    #sensor.set_framesize(sensor.QVGA)
    #sensor.run(1)
    lcd.init()

    img = image.Image('/sd/L.jpg')
    lcd.display(img)
    a = ArrowRec(img,(10, 83))
    b = a.rec_arrow()
    print(b)
    try:
        img.draw_string(0, 0, b, color = (255, 255,255), scale = 3,mono_space = False)
    except:
        pass
    lcd.display(a.draw_blob())
    print(b)

