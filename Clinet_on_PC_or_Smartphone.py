import socket
import numpy
DEFAULT_TARGET_IP='169.254.254.254'



from kivy.app import App
from kivy.uix.button import Button
#from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.core.window import Window
#from kivy.base import runTouchApp
#from kivy.uix.widget import Widget
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.uix.textinput import TextInput

    

class Game(App):


    def build(self):
        self.ip = DEFAULT_TARGET_IP
        self.ncs = NetSocket(8000)
        self.ncr = NetSocket(8090)
        
        self.img=Image(size=(320,240))


        self.connected=False
        self.valid_keys=['w','a','s','d','z','x','c','v','b']
        self.moving_keys=['w','a','s','d']
        #Clock.schedule_once(self.connect,0.5)
        Clock.schedule_interval(self.update, 1/24)
        self.keys=[]
        buttoncolor = (1, 2, 3, 2.5)
        blue = (0.25, 0.8, 1.5, 2.5)
        red = (2.5, 0, 0, 1.5)
        green = (0, 2.5, 0, 1.5)

        Window.bind(on_key_down=self.keydown_action)

        Window.bind(on_key_up=self.keyup_action)
        
        # layouts :

        mainlayout = BoxLayout(orientation='vertical',padding=10)
        mainvertlayout=BoxLayout(orientation='vertical')
        arrowslayout=BoxLayout(orientation='horizontal')
        updownlayout=BoxLayout(orientation='vertical')
        modelayout=BoxLayout(orientation='horizontal')
        
        # buttons :

        btn_left =  Button(id='a',text='left', background_color=buttoncolor, font_size=50)        
        btn_right =  Button(id='d',text='right', background_color=buttoncolor, font_size=50)        
        btn_up =  Button(id='w',text='up', background_color=buttoncolor, font_size=50)        
        btn_down =  Button(id='s',text='down', background_color=buttoncolor, font_size=50)        
        
        btn_mode1 =  Button(id='z',text='keyboard', background_color=buttoncolor, font_size=20)        
        btn_mode2 =  Button(id='x',text='red color detection', background_color=red, font_size=20,text_size= (btn_mode1.width, None))        
        btn_mode3 =  Button(id='c',text='green color detection', background_color=green, font_size=20,text_size= (btn_mode1.width, None))        
        btn_mode4 =  Button(id='v',text='blue color detection', background_color=blue, font_size=20,text_size= (btn_mode1.width, None))        
        btn_mode5 =  Button(id='b',text='face detection', background_color=buttoncolor, font_size=20,text_size= (btn_mode1.width, None))        
        
        btn = [btn_left,btn_right,btn_up,btn_down,btn_mode1,btn_mode2,btn_mode3,btn_mode4,btn_mode5]

        for b in btn:
            b.bind(on_press=self.pressed)
            b.bind(on_release=self.released)

        ip_input = TextInput(text='169.254.254.254',multiline=False)
        ip_input.bind(on_text_validate=self.on_enter)

        
        updownlayout.add_widget(btn_up)
        updownlayout.add_widget(btn_down)

        arrowslayout.add_widget(btn_left)
        arrowslayout.add_widget(updownlayout)
        arrowslayout.add_widget(btn_right)
        
        modelayout.add_widget(btn_mode1)
        modelayout.add_widget(btn_mode2)
        modelayout.add_widget(btn_mode3)
        modelayout.add_widget(btn_mode4)
        modelayout.add_widget(btn_mode5)

        mainlayout.add_widget(self.img)
        
        mainlayout.add_widget(arrowslayout)
        mainlayout.add_widget(modelayout)
        mainlayout.add_widget(ip_input)
        print('ok')
        return mainlayout



    def on_enter(self,instance):
        self.ip=instance.text
        print('User pressed enter in', 'value :',instance.text)
        self.connect(self)


    def pressed(self, button):
        try:
            if(self.connected):
                key=button.id
                print(button.id , "button touched")
                if not(key in self.keys):
                    self.ncs.sendCommand(key)
                    self.keys.append(key)
        except socket.error:
            print('error in pressed')

    def released(self, button):
        try:
            if(self.connected):
                key=button.id
                print(button.id , "button released")
                if (key in self.keys):
                    self.keys.remove(key)
                    if(key in self.moving_keys):
                        key=self.checkKeyUp(key)
                        self.ncs.sendCommand(key)
        except socket.error:
            self.connected=False
            print('error in released')

    def keydown_action(self, keyboard, keycode, text, modifiers,temp):
        try:
            if(self.connected):
                key=chr(keycode)
                print ("got a keydown event: ",chr(keycode))
                if(key in self.valid_keys):
                    if not(key in self.keys):
                        self.ncs.sendCommand(key)
                        self.keys.append(key)
        except socket.error:
            self.connected=False
            print('error in keydown')

    def keyup_action(self, keyboard, keycode,temp):
        try:
            if(self.connected):
                key=chr(keycode)
                print ("got a keyup event: ",chr(keycode))
                if (key in self.keys):
                    self.keys.remove(key)
                    if(key in self.moving_keys):
                        key=self.checkKeyUp(key)
                        self.ncs.sendCommand(key)
        except socket.error:
            self.connected=False
            print('error in keyup')
                
        

    def checkKeyUp(self,key):
        if(key=='w'):
            return 't'
        elif(key=='s'):
            return 'g'
        elif(key=='d'):
            return 'h'
        elif(key=='a'):
            return 'f'
        else:
            return key


    def update(self,event):
        try:

            frameready,frame=self.ncr.recieveFrame()
            
            if frameready:


                print('i want to show')
                image = numpy.rot90(numpy.swapaxes(frame, 0, 1))
                texture = Texture.create(size=(image.shape[1], image.shape[0]), colorfmt="bgr")
                texture.blit_buffer(image.tostring(),colorfmt="bgr", bufferfmt="ubyte")
                self.img.texture=texture
        except socket.error as serr:
            print ('socket error in update')
            self.connected=False
                

    def connect(self,event):
        try:
            print('seeking connection')
            self.ncr.connect(self.ip)
            self.ncs.connect(self.ip)
            self.connected=True
        except socket.error:
            self.connected=False
            print('connection error')
        
    def on_close(self):
        try:
            self.ncr.shutdown(socket.SHUT_RDWR)
            self.ncr.close()
            self.ncs.shutdown(socket.SHUT_RDWR)
            self.ncs.close()
        except socket.error:
            print("socket error")
                

class NetSocket:
    
    def __init__(self,port):
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# Create a socket object
        self.host=''
        self.port=port
        self.remaining=bytearray()
        self.connected=False
    def connect(self,host):
        self.host=host
        print(self.host)
        self.conn.connect((self.host, self.port))
        self.connected=True
        
    def recieveFrame(self):
        data = bytearray()
        ret=False
        frame=[]
        if(self.connected):
            data+=self.remaining
            while(1):
                data+=self.conn.recv(240*320*3)
                if(len(data) >= 230400):
                    frame_data=data[:230400]
                    print(len(frame_data))
                    self.remaining=data[230400:]
                    frame = numpy.frombuffer (frame_data,dtype=numpy.uint8)
                    frame = frame.reshape (240,320,3)
                    ret=True
                    break

        return ret,frame
        
    def sendCommand(self,command):
        self.conn.send(command.encode('UTF8'))


game = Game()
game.run()


