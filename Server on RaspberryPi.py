import socket
import picamera
import picamera.array
import pickle
import cv2
import select
import RPi.GPIO as GPIO
import time
import threading
import fcntl
import struct
import subprocess
from RPLCD import CharLCD

GREENLOWER = (29,86,6)
GREENUPPER = (64,255,255)

REDLOWER = (30,150,50)
REDUPPER = (255,255,180)

BLUELOWER = (110,150,150)
BLUEUPPER = (130,255,255)

DEFAULT_CASCADE_INPUT_PATH = 'haarcascade_frontalface_alt.xml'

class RangeSensor:
    def __init__(self):
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(12, GPIO.LOW)

    def sendTrigger(self):
        GPIO.output(12,GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(12,GPIO.LOW)

    def getEcho(self):
        timeout=0
        while(not (GPIO.input(22))):
            time.sleep(0.000001)
            timeout+=1
            if(timeout>500):
                return 0
        us=0
        while(GPIO.input(22)):
            time.sleep(0.000001)
            us+=1
        return us
    def getRange(self):
        self.sendTrigger()
        us=self.getEcho()
        return us

        
class Object:
    def __init__(self,center,radius):
        self.center=center
        self.radius=radius

class myCamera:
    def __init__(self,width,height):
        self.faceCascade = cv2.CascadeClassifier(DEFAULT_CASCADE_INPUT_PATH)
        self.width=width
        self.height=height
        self.camera=picamera.PiCamera()
        self.camera.resolution = (width,height)
        self.camera.framerate = 24
        self.frame=None
        self.frameready=False
        
    def capture(self):
        stream=picamera.array.PiRGBArray(self.camera)
        self.camera.capture(stream, 'bgr', use_video_port=True)
        self.frame=stream.array
        self.frameready=True
        cv2.waitKey(1)
        stream.seek(0)
        stream.truncate()
        return self.frame

    def findColor(self,frame,colorlower,colorupper):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			
        # construct a mask for the color "green", then perform (handles the actual localization of the green ball in the frame)
        # a series of dilations and erosions to remove any small
        # blobs left in the mask

        self.colorlower = colorlower
        self.colorupper = colorupper
	
        mask = cv2.inRange(hsv, self.colorlower , self.colorupper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        
        # only proceed if at least one contour was found
        radius=0
        center=(0,0)
        if len(cnts) > 0:
        
                # find the largest contour in the mask
                c = max(cnts, key=cv2.contourArea)
                
                # compute the minimum enclosing circle of the largest contour
                ((x,y), radius) = cv2.minEnclosingCircle(c)
                
                # compute centroid
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]) , int(M["m01"] / M["m00"]))
                
                # only proceed if the radius meets a minimum size
                if radius > 1:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0,255,255), 2)
                        cv2.circle(frame, center, 5, (0,0,255), -1)
                return True,Object(center,radius),frame
        return False,Object(center,0),frame

    def findFace(self,frame):

            
        screenColor = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #   customize how the cascade detects your face
        faces = self.faceCascade.detectMultiScale(
            screenColor,
            scaleFactor = 1.1,
            minNeighbors = 1, #number of faces can be detected
            minSize = (5,5), #minimum size of rectangle that is drawn around the face
            flags = cv2.CASCADE_SCALE_IMAGE)


        if len(faces) == 0:
            return False,Object((0,0),0),frame

        elif len(faces) > 0:
            print('Face Detected')

        for (x,y,w,h) in faces:
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
            return True,Object((x+w/2,y+h/2),w/2),frame


    def getObject(self,mode):
        if(mode==2):
            return self.findColor(self.frame,REDLOWER,REDUPPER)
        elif(mode==3):
            return self.findColor(self.frame,GREENLOWER,GREENUPPER)
        elif(mode==4):
            return self.findColor(self.frame,BLUELOWER,BLUEUPPER)
        elif(mode==5):
            return self.findFace(self.frame)

    
    def release(self):
        self.camera.close()
                
class netSocket:
    def __init__(self,port):
        self.server_socket =socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0',port))
        self.commands=[]
        self.conn=False
        
    def waitForConnection(self,number):
        self.server_socket.settimeout(10)
        self.server_socket.listen(number)
        self.conn,self.addr = self.server_socket.accept()
        self.server_socket.settimeout(0)

    def sendFrame(self,frame):
        #data = pickle.dumps(frame)
        d = frame.flatten ()
        data = d.tostring ()
        print(len(data))
        self.conn.sendall(data)

    def recieveCommand(self,car):
        self.threadalive=True
        self.commands=[]
        #fcntl.fcntl(self.conn, fcntl.F_SETFL, os.O_NONBLOCK)
        #self.conn.setblocking(0)
        #self.server_socket.settimeout(1)
        try:
            while(self.threadalive):
            #timeout_in_seconds = 0.001
            #ready = select.select([self.conn], [], [], timeout_in_seconds)
            #if(ready[0]):
                #print('recieving')
                command=self.conn.recv(256)
                #command = pickle.loads(command)
                command=command.decode()
                commands=list(command)
                if commands:
                    self.checkCommand(commands,car)
        except socket.error as serr:
                print(serr.errno)
            
    def stopThread(self):
        self.threadalive=False



    def checkCommand(self,commands,obj):
        print('command')
        print(commands)
        if(type(obj=="<class '__main__.Car'>")):
            print('car class')
            for command in commands:
                command = command.encode('utf8')
                print command
                if command=='z' :
                    obj.changeMode(1)
                    obj.stop()
                
                elif command=='x' :
                    obj.changeMode(2)
                    obj.stop()

                elif command=='c' :
                    obj.changeMode(3)
                    obj.stop()

                elif command=='v' :
                    obj.changeMode(4)
                    obj.stop()
                
                elif command=='b' :
                    obj.changeMode(5)
                    obj.stop()
                    
                elif command=='q':
                    obj.stop()
                    
                elif command=='d':
                    obj.go('d')
                    print('car go right')
                elif command=='a':
                    obj.go('a')
                    print('car go left')
                elif command=='w':
                    obj.go('w')
                    obj.forward=True
                    print('car go forward')
                elif command=='s':
                    obj.go('s')
                    print('car go back')
                
                if command=='h':
                    obj.wait('d')
                    print('car stop right')
                elif command=='f':
                    obj.wait('a')
                    print('car stop left')
                elif command=='t':
                    obj.wait('w')
                    obj.forward=False
                    print('car stop forward')
                elif command=='g':
                    obj.wait('s')
                    print('car stop back')

            
        
    def close(self):
        if(self.conn):
            self.conn.shutdown(socket.SHUT_RDWR)
            self.conn.close()
        if(self.server_socket):
            self.server_socket.shutdown(socket.SHUT_RDWR)
            self.server_socket.close()
            

            
            

class Car:

    def __init__(self):
        GPIO.setup(16, GPIO.OUT)
        GPIO.setup(18, GPIO.OUT)
        GPIO.setup(31, GPIO.OUT)
        GPIO.setup(29, GPIO.OUT)
        self.stop()
        self.speed=0
        self.automatic=False
        self.movement=[]
        self.forward=False
        self.mode=1
    def stop(self):
        GPIO.output(31,True)
        GPIO.output(16,True)
        GPIO.output(18,True)
        GPIO.output(29,True)
    def setSpeed(self,speed):
        self.speed=100-speed
    
    
    def go(self,directions):
        
        for direct in directions:
            if(direct == 'd'):
                GPIO.output(16,False)
            elif(direct == 'a'):
                GPIO.output(18,False)
            elif(direct == 'w'):
                GPIO.output(29,False)
                #GPIO.output(5,False)
            elif(direct == 's'):
                GPIO.output(31,False)
            #self.movement.append(direct)#append what is in direct variable to movement array

    def wait(self,directions):
        for direct in directions:
            if(direct == 'd'):
                GPIO.output(16,True)
            elif(direct == 'a'):
                GPIO.output(18,True)
            elif(direct == 'w'):
                GPIO.output(29,True)
                #GPIO.output(5,True)
            elif(direct == 's'):
                GPIO.output(31,True)
            #self.movement.remove(direct)#remove what is in direct variable from movement array

    def auto(self,camera):
        self.threadalive=True
        RS=RangeSensor()
        waitforobstacle=False
        while(self.threadalive):
            frame=camera.capture()
            if not (waitforobstacle):
                if(self.automatic):
                    res,obj,tmpframe=camera.getObject(self.mode)
                    if(res):
                        camera.frame=tmpframe
                        # *************** self auto *******************
                        (x,y)=obj.center
                        print(x,y,obj.radius)
                        if((obj.radius<camera.width/2) and (obj.radius>5)):
                            self.go('w')
                            if(x<camera.width/3):
                                self.setSpeed(100)
                                self.wait('d')
                                self.go('a')
                                t=float(camera.width-x)/float(camera.width*2)
                                print('t = ',t)
                                time.sleep(t)
                                self.stop()
                                print('auto go left')
                            elif(x>((2*camera.width)/3)):
                                self.setSpeed(100)
                                self.wait('a')
                                self.go('d')
                                t=float(x)/float(camera.width*2)
                                print('t = ',t)
                                time.sleep(t)
                                self.stop()
                                print('auto go right')
                            else:
                                self.setSpeed(100)
                                t=float(abs(x-camera.width/2))/float(camera.width)
                                print('t = ',t)
                                time.sleep(abs(0.4-t))
                                self.stop()
                                self.wait('a')
                                self.wait('d')
                        else:
                            self.stop()
                        #self.auto(obj,camera)
                        # *************** end of self auto **************
                        frame=tmpframe
                    else:
                        self.stop()
            
            
            distance=RS.getRange()
            print(distance)
            print('self.threadalive',self.threadalive)
            if(distance<70):
                waitforobstacle=True
                self.wait('w')
                print ('\n self stop for obstacle\n')
            else:
                if(self.forward and waitforobstacle):
                    self.go('w')
                waitforobstacle=False

    def stopThread(self):       
        self.threadalive=False
        

    def changeMode(self,mode):
        if(mode==1):
            self.setSpeed(100)
            self.automatic=False
        else:
            self.automatic=True
            self.mode=mode

                
                

                
        
        
#end of car
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

def lcd_init():
    lcd = CharLCD(cols=16, rows=2, pin_rs=35, pin_rw=8, pin_e=33, pins_data=[40,38,36,32])
    return lcd

def showIP(lcd):
    lcd.home()
    interfaces = ['eth0','wlan0']
    i=0
    for ifname in interfaces:
        try:
            lcd.cursor_pos = (i, 0)
            lcd.write_string(str(socket.gethostbyname(get_ip_address(ifname))))
            i+=1
        except:
            lcd.write_string(ifname[:len(ifname)-1] + ' not connect')
            i+=1


def GPIO_init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(37, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    
def GPIO_final(car):
    GPIO.cleanup()


def reboot(event):
    print('**************** reboot *****************')
    proc = subprocess.Popen('sudo reboot',shell=True, stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)

def main():
    lcd=lcd_init()
    GPIO_init()
    GPIO.add_event_detect(37, GPIO.FALLING, callback=reboot, bouncetime=300)
    
    while True:
        try:
            ncr=False
            ncs=False
            car=False
            threads=[]
            print('gpio init is done')
            camera=myCamera(320,240)
            print('camera is connected')
            ncr=netSocket(8000)
            ncs=netSocket(8090)
            showIP(lcd)
            car=Car()
            print('car is created')
            
            ncs.waitForConnection(0)
            ncr.waitForConnection(0)
            obj=Object((0,0),0)


        
            
            thr=threading.Thread(target=ncr.recieveCommand,args=(car,))
            threads.append(thr)
            thr.start()

            tha=threading.Thread(target=car.auto,args=(camera,))
            threads.append(tha)
            tha.start()

        
            
            while True:
                if(camera.frameready):
                    ncs.sendFrame(camera.frame)
                    
                    #raise NameError('close program')
                    
        except NameError:
            print("some error")
        except socket.error,msg:
            print('socket error in try 1: ',msg)
        finally:
            print('\n*********************** stoped for error *************************\n')
            if(car):
                car.stop()
                car.stopThread()
            try:
                if(ncr):
                    ncr.stopThread()
                    ncr.close()
                if(ncs):
                    ncs.close()
            except socket.error,msg:
                print('socket error in try 2: ',msg)
            for thread in threads:
                thread.join()
            camera.release()
            #if(car):
                #GPIO_final(car)
                #cv2.destroyAllWindows()
           
        


if __name__ == '__main__':
    main()
