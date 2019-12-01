import sensor, image, time, math
from pyb import SPI,Pin
import json

spi = SPI(2, SPI.MASTER, baudrate=281250, polarity=0, phase=0)
cs  = Pin("P3", Pin.OUT_PP)
def spi_write(data):
    if data:
        for d in data:
            cs.low()
            spi.send(d)
            cs.high()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # grayscale is faster
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 1000)
clock = time.clock()
grayscale_thres = (135,255)
thres = (0,50)

# 所有线段都有 `x1()`, `y1()`, `x2()`, and `y2()` 方法来获得他们的终点
# 一个 `line()` 方法来获得所有上述的四个元组值，可用于 `draw_line()`.

#通过k_inv计算倾斜角[0,180)
def kToAngle(k_inv):
    angle = math.degrees(math.atan(k_inv)) + 90
    return angle

num = 13
#horiz_found = [False] * num
k = 0
angle = 0
Isfound = False
IsEmpty = True
y_pos = 0
x_pos = 0

while(True):
    clock.tick()
    img = sensor.snapshot()
    img.binary([grayscale_thres])
    img.dilate(2)
    img.erode(4)
    img.dilate(3)

    y_list = []         #横线y坐标列表
    blobcenter_x = []   #纵线x坐标列表
    blobcenter_y = []   #纵线y坐标列表

    IsFound = False
    IsEmpty = True
    found_cnt = 0
    pixels_total = 0
    for i in range(0,num):
        blobs = img.find_blobs([thres],roi = [0,3*i,160,3],pixels_threshold = 40,margin = 0)
        if blobs:
            for blob in blobs:
                pixels_total += blob.pixels()

            if len(blobs) > 1 or pixels_total > 150:
                found_cnt+=1
                #img.draw_cross(blobs[0][5], blobs[0][6])
                y_list.append(3*(i+0.5))
            else:
                #img.draw_string(blobs[0][5], blobs[0][6], str(i))
                x = blobs[0][5]
                y = blobs[0][6]
                blobcenter_x.append(x)
                blobcenter_y.append(y)
            pixels_total = 0
    if found_cnt > 1:
        IsFound = True

    #纵线倾斜角
    count = len(blobcenter_x)

    angle_list = [0]*(count+1)  #相邻点连线倾斜角
    angle_list[0] = 0          #头尾哨兵
    angle_list[count] = 0
    usable = [True]*count

    if count > 1:
        IsEmpty = False
        sum_angle = 0
        for i in range(1,count):
            k_temp_inv = (blobcenter_x[i] - blobcenter_x[i-1])/(blobcenter_y[i] - blobcenter_y[i-1])
            angle_list[i] = kToAngle(k_temp_inv)
            sum_angle += angle_list[i]
        ave_angle = sum_angle / (count - 1)
        #标准差
        delta_sum = 0
        for i in range(1,count):
            delta_sum += (angle_list[i] - ave_angle)**2
        s = math.sqrt(delta_sum / count)
        for i in range(0,count):
            if abs(angle_list[i] - ave_angle) > 1.2*s and abs(angle_list[i+1] - ave_angle) > 1.2*s:
                usable[i] = False

        #最小二乘法
        sum_x = 0
        sum_y = 0
        sum_xx = 0
        sum_xy = 0
        num_u = 0
        for i in range(0,count):
            if usable[i] == True:
                sum_x += blobcenter_x[i]
                sum_y += blobcenter_y[i]
                sum_xx += blobcenter_x[i]**2
                sum_xy += blobcenter_x[i]*blobcenter_y[i]
                num_u += 1
        if num_u * sum_xy - sum_x * sum_y:
            k_inv = (num_u * sum_xx - sum_x**2)/(num_u * sum_xy - sum_x * sum_y)
            angle = kToAngle(k_inv)
        else:
            angle = 90

    #横线y坐标平均值z
    sum_temp = 0
    if y_list:
        for i in y_list:
            sum_temp += i
        y_pos = sum_temp/len(y_list)

    #竖线x坐标平均值
    sum_temp = 0
    cnt = 0
    if blobcenter_x:
        for i in range(len(blobcenter_x)):
            if usable[i] == True:
                sum_temp += blobcenter_x[i]
                cnt+=1
        if cnt:
            x_pos = sum_temp/cnt
        else:
            x_pos = 80

    #uart send
    sendlist = bytearray([0xff,0xfe,IsEmpty + (IsFound << 1),int(x_pos),int(angle),int(y_pos)])
    #print('x',x_pos)
    #print('y',y_pos)
    #print('a',angle)
    #print(IsEmpty,IsFound,found_cnt)
    spi_write(sendlist)

