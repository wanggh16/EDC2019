import sensor, image, time, math
from pyb import Pin, Timer

tim = Timer(4, freq=1000) # Frequency in Hz
# 生成1kHZ方波，使用TIM4，channels 1 and 2分别是 50% 和 75% 占空比。
ch1 = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=100)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # grayscale is faster
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 1000)
clock = time.clock()
grayscale_thres = (100,255)
thres = (0,50)

while(True):
    clock.tick()
    img = sensor.snapshot()
    img.binary([grayscale_thres])

    blobs = img.find_blobs([thres],roi = [0,51,160,3],pixels_threshold = 20,margin = 0)
    pixels_total = 0
    max_pixels = 0
    max_index = 0
    i = 0
    x = 240
    if blobs:
        for blob in blobs:
            pixels_total += blob.pixels()
            if blob.pixels() > max_pixels:
                max_pixels = blob.pixels()
                max_index = i
            i += 1
        if pixels_total < 150:
            x = blobs[max_index].cx()

    ch1 = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0.4*x)
    print('x',x)

