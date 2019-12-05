import sensor, image, time, math
from pyb import UART

uart=UART(3,115200)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # grayscale is faster
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 1000)
clock = time.clock()
grayscale_thres = (90,255)
thres = (0,50)

while(True):
    clock.tick()
    img = sensor.snapshot()
    img.binary([grayscale_thres])

    blobs = img.find_blobs([thres],roi = [0,63,160,3],pixels_threshold = 20,margin = 0)
    pixels_total = 0
    max_pixels = 0
    max_index = 0
    i = 0
    x = 250
    if blobs:
        for blob in blobs:
            pixels_total += blob.pixels()
            if blob.pixels() > max_pixels:
                max_pixels = blob.pixels()
                max_index = i
            i += 1
        if pixels_total < 150:
            x = blobs[max_index].cx()

    #uart send
    sendlist = bytearray([0xff,0xfe,int(x)])
    print('x',x)
    uart.write(sendlist)

