
# Based on code by Dave Copeland and Xander Hostler

import serial
import time, os, os.path
import RPi.GPIO as GPIO

class uCAM_III(object):

    # BCM number for reset pin on GPIO header
    default_resetPin = 23
    # time to wait before reading camera response
    default_responseDelay = 1
    # time to wait for reading camera reponse to sync
    default_syncResponseDelay = 1
    # time to wait between commands
    default_interCommandDelay = 1

    # uCAM III protocol...
    INITIAL = (0xAA,0x01,0x00,0x00,0x00,0x00)
    # INITIAL_JPEG = (0xAA,0x01,0x00,0x07,0x07,0x03)  # 160x128 format      
    # INITIAL_JPEG = (0xAA,0x01,0x00,0x07,0x07,0x05)  # 320x240 format      
    INITIAL_JPEG = (0xAA,0x01,0x00,0x07,0x07,0x07)  # 640x480 format      
    SET_PKG_SIZE = (0xAA,0x06,0x08,0x00,0x02,0x00)
    SNAPSHOT = (0xAA,0x05,0x00,0x00,0x00,0x00)
    GETPICTURE = (0xAA,0x04,0x01,0x00,0x00,0x00)
    FULL_RESET = (0xAA,0x08,0x00,0x00,0x00,0x00)
    SYNC = (0xAA,0x0D,0x00,0x00,0x00,0x00)
    ACK = (0xAA,0x0E)
    LIGHT = (0xAA,0x13,0x01,0x00,0x00,0x00)
    EXPSETTINGS = (0xAA,0x14,0x02,0x02,0x02,0x00)

    pkgSize = 512

    def __init__(self,**kw):
        self.resetPin = kw.get('resetPin',self.default_resetPin)
        self.responseDelay = kw.get('responseDelay',self.default_responseDelay)
        self.syncResponseDelay = kw.get('syncResponseDelay',self.default_syncResponseDelay)
        self.interCommandDelay = kw.get('interCommandDelay',self.default_interCommandDelay)
        self.verbose = kw.get('verbose',False)
        GPIO.setup(self.resetPin,GPIO.OUT)

    def reset(self):
        GPIO.output(self.resetPin,GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(self.resetPin,GPIO.HIGH)

    def full_reset(self):
        self.command(self.FULL_RESET)

    def command(self,bytes,**kw):
        responseDelay = kw.get('responseDelay',self.responseDelay)
        verbose=kw.get('verbose',self.verbose)
        wait=kw.get('wait',True)
        if verbose:
            print("Write:",len(bytes),":"," ".join(map(lambda b: "%02X"%b,bytes)))
        self.write(bytes)
        time.sleep(responseDelay)
        retval = self.read()
        while wait and len(retval) == 0:
            retval = self.read()
        if verbose:
            print("Read: ",len(retval),":"," ".join(map(lambda b: "%02X"%b,retval)))
        return retval

    def check_ack(self,reply,cmd):
        return tuple(map(int,reply[:3])) == (0xAA,0x0E,cmd)

    def check_sync_ack(self,reply):
        return self.check_ack(reply,0x0D)

    def sync(self,**kw):
        syncResponseDelay = kw.get('syncResponseDelay',self.syncResponseDelay)
        maxTries = 60
        currentTry = 0
        reply = b''
        delayValue = 0.005
        delayIncrement = 0.001
    
        while (currentTry < maxTries) and (len(reply) != 12 or not self.check_sync_ack(reply)):
            # print("Sync attempt:",currentTry)
            delayValue += delayIncrement
            reply = self.command(self.SYNC, responseDelay=syncResponseDelay, wait=False)
            currentTry +=1

        if len(reply) != 12 or not self.check_sync_ack(reply):
            # Didn't work
            print("Sync didn't work. Sent %d tries" % currentTry)
            return False

        # it did work
        print("Camera replied", reply, "attempts:",currentTry)
        # send the ACK back to camera        
        bytes = self.ACK+(0x0D,0x00,0x00,0x00)
        if self.verbose:
            print("Write:",len(bytes),":"," ".join(map(lambda b: "%02X"%b,bytes)))
        response = self.write(bytes)
        return True

    def takephoto(self):
        reply = self.command(self.INITIAL_JPEG)
        time.sleep(self.interCommandDelay)
        reply = self.command(self.SET_PKG_SIZE)
        time.sleep(self.interCommandDelay)
        reply = self.command(self.EXPSETTINGS)
        time.sleep(self.interCommandDelay)
        reply = self.command(self.SNAPSHOT)

    def retrieveDataFromPkg(self,pkg):
        # pkgID = pkg[0:2]
        dataBuffer = pkg[4:-2]
        return dataBuffer

    def getPictureSize(self):
        response = self.command(self.GETPICTURE)
        print("Wrote GETPICTURE. Camera replied",response)
        return response[-1]*256*256+response[-2]*256+response[-3]

    def getPicture(self):
        camReply = self.command(self.GETPICTURE)
        # print("Wrote GETPICTURE. Camera replied",camReply)

        replyLen=len(camReply)
        imageSize = camReply[replyLen-1]*256*256+camReply[replyLen-2]*256+camReply[replyLen-3]
        print("Picture size:",imageSize,"bytes")

        numPackages = int(imageSize/(self.pkgSize-6))+1
        # start the download by ACKing back
        pkgCounter0 = 0
        pkgCounter1 = 0  # I'm cheating and assuming num packages < 255!!
        byteCounter = 0
        pictureArray = b''

        for indx in range(0,numPackages):
            pkgCounter0 = indx
            ackResponse = self.ACK+(0,0,pkgCounter0,pkgCounter1)
            while True:
                pkg = self.command(ackResponse)
                pkgID0 = pkg[0]
                pkgID1 = pkg[1]
                print("Received package %d %d - size %d bytes" % (pkgID1,pkgID0,len(pkg)))
                # the only tell for bad packet we have is if the package is short
                if len(pkg) == min(self.pkgSize,imageSize-len(pictureArray)+6):
                    break

            pictureArray += self.retrieveDataFromPkg(pkg)
            print("Received %6.2f%%" % (100.0*len(pictureArray)/imageSize,))

        print("Received - %d  bytes" % len(pictureArray))    
        return pictureArray

    def savePicture(self,filename):
        print("Picture filename:",filename)
        start = time.time()
        picture = self.getPicture()
        print("Picture size:",len(picture))
        picFile = open(filename,'wb')
        picFile.write(picture)
        picFile.close()
        print("Time to save: %0f sec"%(time.time()-start,))

class uCAM_III_Serial(uCAM_III):
    # Constants
    default_baudRate = 56000

    # initialize
    def __init__(self,serialDevice,**kw):
        baudRate = kw.get('baudRate',self.default_baudRate)
        self.serialPort = serial.Serial(serialDevice,baudRate)
        super(uCAM_III_Serial,self).__init__(**kw)

    def read(self):
        readString = b''
        bytesToRead = self.serialPort.in_waiting
        while bytesToRead >0:
            readByte= self.serialPort.read(1)
            readString += readByte
            bytesToRead = self.serialPort.in_waiting
            time.sleep(0.00005)
        return readString

    def write(self,bytes):
        self.serialPort.write(bytes)

import max3100

class uCAM_III_MAX3100(uCAM_III):
    # Constants
    # default_baudRate = 38400
    # default_baudRate = 19200
    default_baudRate = 9600

    # initialize
    def __init__(self,spiDevice,**kw):
        baudRate = kw.get('baudRate',self.default_baudRate)
        # other MAX3100 parameters are set to default values
        self.max3100 = max3100.MAX3100(spiif=spiDevice[0],
                                       spiselect=spiDevice[1],
                                       baud=baudRate)
        super(uCAM_III_MAX3100,self).__init__(**kw)

    def read(self):
        return self.max3100.readbytes()

    def write(self,bytes):
        self.max3100.writebytes(bytes)

class Button(object):

    default_buttonPin = 24

    def __init__(self, **kw):
        self.buttonPin = kw.get('buttonPin',self.default_buttonPin)
        self.callback = kw.get('callback',self.callback)
        GPIO.setup(self.buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.buttonPin,GPIO.RISING,callback=self.callback)

    def callback(self,channel):
        print("Button was pushed!")

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

button_pressed = True

def button_callback(channel):
    global button_pressed
    button_pressed = True
    print("Button was pushed!")

button = Button(buttonPin=24, callback=button_callback)

connection = "serial"
# connection = "max3100"

if connection == "serial":

    camera = uCAM_III_Serial(serialDevice="/dev/serial0", 
                             resetPin=23,
                             responseDelay=0.2,
                             syncResponseDelay=0.05)

elif connection == "max3100":

    camera = uCAM_III_MAX3100(spiDevice=(0,0), 
                              resetPin=23,
                              responseDelay=0,
                              syncResponseDelay=0,
                              verbose=True)

counter = 1
while True:
    if button_pressed:
        print("Connect w/ camera")
        camera.reset()
        assert camera.sync()
        time.sleep(2)

        print("Take photo")
        camera.takephoto()

        print("Save photo")
        filename = 'testPic-%03d.jpg'%(counter,)
        camera.savePicture(filename)
        counter += 1

        button_pressed = False

    time.sleep(.1)

GPIO.cleanup()

