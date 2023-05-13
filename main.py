import sensor, image, time, json, pyb , os, tf, uos, gc
from pid import PID
from pyb import UART,Pin, Timer
THRESHOLD = (2, 41, -29, 19, -16, 23) # 黑线
uart = UART(3, 115200)# p4为TX,p5为RX
sensor.reset()
sensor.set_vflip(True)#镜像翻转
sensor.set_hmirror(False)
sensor.set_transpose(True)   #图像旋转转90°
sensor.set_pixformat(sensor.RGB565) # RGB565模式
sensor.set_framesize(sensor.QQVGA) # 120*160分辨率
sensor.skip_frames(time=1000)
clock = time.clock()
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))# 照明模块50kHz pin6 timer2 channel1
light.pulse_width_percent(25) # 控制亮度 0~100
rho_pid = PID(p=-20, i=0,d=-4.5)   # rho是直线到图像正中心的距离
#rho_pid = PID(p=0, i=0,d=0)
#theta_pid = PID(p=1.125, i=0,d=0.5)
theta_pid = PID(p=45, i=0,d=10)
global cross#T字路口标志位
cross = 0 #抵消一个

last_detection_time =pyb.millis()  # 上次检测时间毫秒


#flag = True#开始主循环标志1
#while flag:
    #while uart.any():
        #data = uart.readline().decode().strip()
        #print(data)
        #if data == '1':
            #flag = False
            #break

#pyb.delay(3000)#####################################################
#基础主循环
while(True):
    clock.tick()#开始追踪运行时间。
    sensor.set_framesize(sensor.QQVGA) # 120*160分辨率
    img = sensor.snapshot().binary([THRESHOLD]) # 二值化图像

    ROI = (17,9,65,142)#中央巡线
    img.draw_rectangle(ROI,color=(0, 255, 0))
    ROI_T = (1,87,29,8)#T字形
    img.draw_rectangle(ROI_T,color=(0, 255, 0))
    blobs = img.find_blobs([(100, 100)], roi=ROI, area_threshold=275, merge=True)    # 100,100代表色值max和min，也就是黑线的色值范围
    line = img.get_regression([(100, 100)], roi=ROI, robust=True)
    blobs_T = img.find_blobs([(100, 100)], roi=ROI_T, area_threshold=30, merge=True)
#巡线模块
    if blobs:
        if line:
            rho_err = abs(line.rho()) - 47	# 47为小车摄像头正中间
            if line.theta() > 90:
                theta_err = line.theta() - 180
            else:
                theta_err = line.theta()
            img.draw_line(line.line(), color=127)
            if line.magnitude() > 25 :
                rho_output = rho_pid.get_pid(rho_err, 1)
                theta_output = theta_pid.get_pid(theta_err, 1)
                output1 = rho_output + theta_output + 1600.0
                if output1 >= 2500.0:
                    output1 = 2500.0
                if output1 <= 1000.0:
                    output1 = 1000.0
                obj = [output1]
                output = json.dumps(obj)
                print("1" + output + "\r\n")
                uart.write("1" + output + "\r\n")
            else:	#线太短，停止
                #print(3)
                #uart.write("3"  + "\r\n")
                pass
        else:	#没有找到线
            #print(3)
            #uart.write("3" + "\r\n")
            pass
    # T字路口模块
    if blobs_T:
        current_time = pyb.millis()
        if (current_time - last_detection_time) > 1100   :
            cross += 1  # 十字路口标志位
            last_detection_time = current_time
            obj = [cross]
            output = json.dumps(obj)
            #if cross <= 3:
            print("2" + output + "\r\n")
            uart.write("2" + output + "\r\n")
    else:
        pass



##拓展题识别图像模块—————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
#global bufferq#预置主循环跳出标志“缓冲进入”
#bufferq = 0
#global stopq#预置主循环跳出标志“已有车停入”
#stopq = 0
##加载模型
#net = None
#labels = None
#net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
#labels = [line.rstrip('\n') for line in open("labels.txt")]
#clock = time.clock()
##预置主循环
#while(True):
    #clock.tick()
    #sensor.set_framesize(sensor.QVGA)
    #sensor.set_windowing((240, 240))# Set 240x240 window
    #img = sensor.snapshot()
    #print("**********")
    ## default settings just do one detection... change them to search the image...
    #for obj in net.classify(img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
        #predictions_list = list(zip(labels, obj.output()))
        #if (predictions_list[1][1])>0.900000:
            #print(predictions_list[1][0])#1是车位已满，2是缓冲区
            #bufferq=1;
        #if (predictions_list[0][1])>0.900000:
            #print(predictions_list[0][0])#1是车位已满，2是缓冲区
            #stopq=1;
    #if  stopq==1:#识别到有车停入
        #break
    #if  bufferq==1:#识别到缓冲区
        #break
##———————————————————————————————————————————————————————————————————————————————————————————————————————————

