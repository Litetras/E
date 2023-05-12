#初始化模块—————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
import sensor, image, time, json, pyb , os, tf, uos, gc
from pid import PID
from pyb import UART,Pin, Timer
THRESHOLD = (7, 20, -7, 9, -8, 13) # 黑线
uart = UART(3, 115200)  # p4为TX,p5为RX
sensor.reset()
sensor.set_vflip(True)      # 左右镜像
sensor.set_hmirror(False)    # 上下镜像
sensor.set_transpose(True)   #转90°
sensor.set_pixformat(sensor.RGB565) # 使用RGB565模式
sensor.set_framesize(sensor.QQQVGA) # 80*60分辨率
sensor.skip_frames(time=1000)
clock = time.clock()
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))# 照明模块50kHz pin6 timer2 channel1
light.pulse_width_percent(50) # 控制亮度 0~100
rho_pid = PID(p=-20, i=0)   # rho是直线到图像正中心的距离
theta_pid = PID(p=-0.2, i=0)
global cross#T字路口标志位
cross = 0
#拓展题识别图像模块—————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

global bufferq#预置主循环跳出标志“缓冲区”
bufferq = 0
global stopq#预置主循环跳出标志“停了车”
stopq = 0


#加载模型
net = None
labels = None
net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
labels = [line.rstrip('\n') for line in open("labels.txt")]
clock = time.clock()
#预置主循环
while(True):
    clock.tick()
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing((240, 240))       # Set 240x240 window.
    img = sensor.snapshot()
    print("**********")
    # default settings just do one detection... change them to search the image...
    for obj in net.classify(img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
        #print("**********\nPredictions at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
        #img.draw_rectangle(obj.rect(),color=(0, 255, 0))
        predictions_list = list(zip(labels, obj.output()))
        #for i in range(len(predictions_list)):
            #print("%s = %f" % (predictions_list[i][0], predictions_list[i][1]))
        if (predictions_list[1][1])>0.900000:
            print(predictions_list[1][0])#1是车位已满，2是缓冲区
            bufferq=1;
        if (predictions_list[0][1])>0.900000:
            print(predictions_list[0][0])#1是车位已满，2是缓冲区
            stopq=1;
    if  stopq==1:
        break
    if  bufferq==1:
        break
#——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————



#基础主循环
while(True):
    clock.tick()                    # Update the FPS clock.
    sensor.set_framesize(sensor.QQQVGA) # 80*60分辨率
    img = sensor.snapshot().binary([THRESHOLD]) # 二值化图像
    ROI = (18,1,27,79)#中央巡线
    img.draw_rectangle(ROI,color=(0, 255, 0))
    ROI_T = (0,22,15,40)#T字形
    img.draw_rectangle(ROI_T,color=(0, 255, 0))
    blobs = img.find_blobs([(100, 100)], roi=ROI, area_threshold=15, merge=True)    # 100,100代表色值max和min
    line = img.get_regression([(100, 100)], roi=ROI, robust=True)
    blobs_T = img.find_blobs([(100, 100)], roi=ROI_T, area_threshold=15, merge=True)    # 100,100代表色值max和min
    #for blob in blobs:
        #x, y, w, h = blob.x(), blob.y(), blob.w(), blob.h()
        #img.draw_rectangle(x, y, w, h)
        #print(x, y, w, h)
#巡线模块
    if blobs:
        if line:
            rho_err = abs(line.rho()) - 47	# 47为小车摄像头正中间
            if line.theta() > 90:
                theta_err = line.theta() - 180
            else:
                theta_err = line.theta()
            img.draw_line(line.line(), color=127)
            if line.magnitude() > 8:
                rho_output = rho_pid.get_pid(rho_err, 1)
                theta_output = theta_pid.get_pid(theta_err, 1)
                output = rho_output + theta_output
                obj = [output]
                output = json.dumps(obj)
                print("1" + output + "\r\n")
                uart.write("1" + output + "\r\n")
            else:	#线太短，停止
                print(3)
                uart.write("3"  + "\r\n")
                pass
        else:	#没有找到线
            print(3)
            uart.write("3" + "\r\n")
            pass

    #T字路口模块
    if blobs_T:
        cross += 1	#十字路口标志位
        #print('2')
        print(cross)
        #uart.write("2" + "\r\n")
        #pyb.delay(500)
    else:
        pass










# 照明模块50kHz pin6 timer2 channel1
##light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
##light.pulse_width_percent(50) # 控制亮度 0~100

#rho_pid = PID(p=-20, i=0)   # rho是直线到图像中点的距离
#theta_pid = PID(p=-0.2, i=0)
#global cross    #十字路口标志位

#while True:
    #clock.tick()
    #img = sensor.snapshot().binary([THRESHOLD]) # 二值化图像
    #ROI = (39, 0, 15, 59)  						 # 47为小车摄像头正中间
    #blobs = img.find_blobs([(100, 100)], roi=ROI, area_threshold=15, merge=True)    # 100,100代表色值max和min
    #line = img.get_regression([(100, 100)], roi=ROI, robust=True)
    #ROI_L = (0, 45, 32, 50)
    #ROI_R = (59, 45, 21, 50)
    #blobs1 = img.find_blobs([(100, 100)], roi=ROI_L, area_threshold=15, merge=True)
    #blobs2 = img.find_blobs([(100, 100)], roi=ROI_R, area_threshold=15, merge=True)

    ##试试img.draw_rectangle(ROI)
    #img.draw_line((35, 0, 35, 60), color=(0, 255, 0))   # !注意画的线颜色也会被色块查找函数使用，所以不要画白线
    #img.draw_line((59, 0, 59, 60), color=(0, 255, 0))

    #img.draw_line((0, 45, 35, 45), color=(0, 255, 0))
    #img.draw_line((59, 45, 80, 45), color=(0, 255, 0))
    #if blobs1 and blobs2:
        #cross = 1	#十字路口标志位
        #print('2')
        #uart.write("2" + "\r\n")
        #pyb.delay(500)
    #else:
        #cross = 0
        #pass

    #if blobs:
        #if line:
            #rho_err = abs(line.rho()) - 47	# 47为小车摄像头正中间
            #if line.theta() > 90:
                #theta_err = line.theta() - 180
            #else:
                #theta_err = line.theta()
            #img.draw_line(line.line(), color=127)
            #if line.magnitude() > 8:
                #rho_output = rho_pid.get_pid(rho_err, 1)
                #theta_output = theta_pid.get_pid(theta_err, 1)
                #output = rho_output + theta_output
                #obj = [output]
                #output = json.dumps(obj)
                #print("1" + output + "\r\n")
                #uart.write("1" + output + "\r\n")
            #else:	#线太短，停止
                #print(3)
                #uart.write("3"  + "\r\n")
                #pass
        #else:	#没有找到线
            #print(3)
            #uart.write("3" + "\r\n")
            #pass


