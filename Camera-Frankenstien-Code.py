
# UCAM3_Test.py- test of the 4D Systems uCAM-III camera
# Last update 3/23/2021
#
# Revision history: 6/13/2020 DJC - inception
#                   3/23/2021 DJC - revived experiment


import serial           # import the serial library
import time
import RPi.GPIO as GPIO

# set up the serial port for 56000 baud

baudRate = 56000
resetPin = 11
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

pictureArray = b''
       
def resetCamera():
    GPIO.output(resetPin,GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(resetPin,GPIO.HIGH)
    return

def syncCamera(serPort):
# synchronize the camera serial connection
    maxTries = 60
    currentTry = 0
    reply = b''
    delayValue = 0.004
    delayIncrement = 0.001
    
    while (currentTry < maxTries)&(reply==b''):
        serPort.write(SYNC)
        delayValue += delayIncrement
        time.sleep(delayValue)
        reply = receiveData(serPort)
        currentTry +=1


    if reply==b'':
        # Didn't work
        print("Sync didn't work. Sent %d tries" % currentTry)
        return 0
    else:
        # it did work
        print("Camera replied", reply)
        serPort.write(ACK+(0x0D,0X00,0X00,0X00))    # send the ACK back to camera        
        return currentTry

def receiveData(serPort):
# Receive characters on the serial port until the buffer is empty
    readString = b''

    bytesToRead = serPort.in_waiting

    while bytesToRead >0:
        readByte= serPort.read(1)
        readString += readByte
        bytesToRead = serPort.in_waiting
        time.sleep(0.00005)

    return readString

def takeJPEG(serPort):
    # first shot at taking a JPEG. Does not return the picture.
    timeToWait = 1
    serPort.write(INITIAL_JPEG)
    time.sleep(timeToWait)
    print("Wrote INITIAL_JPEG. Camera replied",receiveData(serPort))
    serPort.write(SET_PKG_SIZE)
    time.sleep(timeToWait)
    print("Wrote SET_PKG_SIZE. Camera replied",receiveData(serPort))
    serPort.write(EXPSETTINGS)
    time.sleep(timeToWait)
    print("Wrote EXPSETTINGS. Camera replied",receiveData(serPort))
    serPort.write(SNAPSHOT)
    time.sleep(timeToWait)
    print("Wrote SNAPSHOT. Camera replied",receiveData(serPort))

def retrieveDataFromPkg(pkg):
    pkgLength = len(pkg)
    dataBuffer = b''
    pkgID = pkg[0:2]
    dataBuffer = pkg[4:pkgLength-2]
    return dataBuffer

    
def getPicture(serPort):
    serPort.write(GETPICTURE)
    time.sleep(1)
    camReply = receiveData(serPort)
    print("Wrote GETPICTURE. Camera replied",camReply)

    replyLen=len(camReply)
    imageSize = camReply[replyLen-1]*256*256+camReply[replyLen-2]*256+camReply[replyLen-3]
    print("Data Length: ",imageSize,"bytes")

    numPackages = int(imageSize/(pkgSize-6))+1
    # start the download by ACKing back
    pkgCounter0 = 0
    pkgCounter1 = 0  # I'm cheating and assuming num packages < 255!!
    byteCounter = 0
    pictureArray = b''

    for indx in range(0,numPackages):
        pkgCounter0 = indx
        ackResponse = ACK+(0,0,pkgCounter0,pkgCounter1)
        serPort.write(ackResponse)
        time.sleep(0.05)
        pkg = receiveData(serPort)
        pkgID0 = pkg[0]
        pkgID1 = pkg[1]
        pctRec = indx/numPackages
        print("Received %6.2f" % pctRec )
        # print("Received package %d %d - size %d bytes" % (pkgID1,pkgID0,len(pkg)))
        pictureArray += retrieveDataFromPkg(pkg)
    print("Received - %d  bytes" % len(pictureArray))    
    return pictureArray



    
# **** MAIN STARTS HERE *****


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(resetPin,GPIO.OUT)

def button_callback(channel):
    print("Button was pushed!")

GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.add_event_detect(12,GPIO.RISING,callback=button_callback)

message = input ("Press enter to quit/n/n")

GPIO.cleanup()


serPort = serial.Serial ("/dev/serial0",baudRate)
print("Serial Port is %s\n" % serPort.name)

print("Resetting Camera")
resetCamera()

time.sleep(1)

print("Syncing Camera")

print("Syncing attempts: ",syncCamera(serPort))

# print("Sending Reset")

# serPort.write(FULL_RESET)

time.sleep(0.01)

# print("Camera replied",receiveData(serPort))

time.sleep(2)
takeJPEG(serPort)
pictureArray = getPicture(serPort)

# Saving file

picFile = open('/home/pi/Desktop/testPic.jpg','wb')
picFile.write(pictureArray)
picFile.close()
