##Author  : Md Asifuzzaman Khan
##Date    : 10/5/2022

import numpy as np
import serial
import cv2
import cv2.aruco as aruco
import time
import math
       
cap = cv2.VideoCapture(0)
cap.set(3,960)     #horizontal pixels
cap.set(4,720)     #vertical pixels
cap.set(5,30)      #Frame Rate

COM='COM7'      #bluetooth COM port no
baud=57600


port=1
Ardu_stat=1    #set it to 0 if you don't wanna connect to bluetooth

drive_val=[0,0,0] # index 0 is for direction, 1 is left wheel speed and 2 is right wheel speed
#index 0 value 0 means both forward
#index 0 value 01 means left forward right reverse
#index 0 value 16 means left reverse right forward
#index 0 value 17 means both reverse
draw=False
arduino=None

currentspeed=35
conf_dist=20
pts = np.empty(shape=[0, 2],dtype=np.int32) #array of points for path creation

if Ardu_stat==1:
    try:
        arduino = serial.Serial(COM, baud)
        print('Port found!')
        print("Press 'L' key to make/break loop and start/stop the bot")
        print("Press 'R' key to undo path")
        print("Press 'Q' key to exit program and stop bot")
        time.sleep(2)
    except serial.serialutil.SerialException:
        print('Port not found!')
        port=0
        
def mouse_event(event, x, y, flags, params): #register a path point only whne left buton is pressed and mouse is moved so that a continous path is created
    global pts,draw
    if event == cv2.EVENT_LBUTTONDOWN:
        draw=True
    elif event == cv2.EVENT_MOUSEMOVE:
        if draw:
            pts=np.append(pts,[[x,y]],axis=0)
    elif event == cv2.EVENT_LBUTTONUP:
        draw=False
        
isclosed=False #flag for closing/opening the path and start/stop the bot

def dist(a,b):
    return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def extend(a,b,length):
    return (b[0]+int((b[0]-a[0])*length/dist(a,b)),b[1]+int((b[1]-a[1])*length/dist(a,b)))

def drive(left,right):
    global drive_val
    dir=0
    if(left<0):
        #dir=16
        left=-0*left
    elif(right<0):
        #dir=1
        right=-0*right
        
    if right>100:  #limiting max speed
        right=100
    if left>100:
        left=100  
    drive_val=[dir,left,right]
        
prev_E=0
E_sum=0
def PID_calc(angle): #PID controller
    global prev_E,E_sum
    if angle>180:
        angle=angle-360
    #PID parameters (fixed through trial and error)
    P=0.1
    I=0.005
    D=0.7
    
    E=(angle-0)
    E_sum=E_sum+E
    E_del=(E-prev_E)
    val=int((P*E)+(I*E)+(D*E_del))
    prev_angle=angle
    
    left=currentspeed-val
    right=currentspeed+val
    drive(left,right)
    
cv2.namedWindow('AR_LFR')
cv2.setMouseCallback('AR_LFR',mouse_event)

i=0
start=False
while(True):
    c1 = cv2.getTickCount() 
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)

    arucoParameters = aruco.DetectorParameters_create()
    corners, _, _ = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters) #check for AR tag
    if corners:
        for corner in corners: #ID dependency can be specified in this section. In this example any ID will do
            frame = aruco.drawDetectedMarkers(frame, [corner])
            x=int((corner[0][2][0]+corner[0][3][0]))//2
            y=int((corner[0][2][1]+corner[0][3][1]))//2

            xm=int((corner[0][0][0]+corner[0][1][0]))//2
            ym=int((corner[0][0][1]+corner[0][1][1]))//2

            cv2.circle(frame, (x,y), 5,(255, 0, 0),2)
            cv2.circle(frame, (xm,ym), 5,(255, 0, 0),2)

            if(len(pts)>0 and start):
                frame = cv2.line(frame,(x,y),extend((x,y),(pts[i][0],pts[i][1]),100),(0,255,0),2)
                angle1 = math.atan2(y-pts[i][1],x-pts[i][0])
                angle2 = math.atan2(y-ym,x-xm)
                raw_angle = math.degrees(angle2-angle1)
                PID_calc((raw_angle+360)%360)             
          
                if(dist((xm,ym),(pts[i][0],pts[i][1]))<conf_dist):
                    i=i+1
                if(i>len(pts)-1):
                    i=0
                try:
                    if dist((xm,ym),(pts[i][0],pts[i][1]))>dist((xm,ym),(pts[i+1][0],pts[i+1][1])):
                        i=i+1
                except:
                    pass
                    
            frame = cv2.line(frame,(x,y),extend((x,y),(xm,ym),100),(0,255,255),2)
#if you want to make the 4 corner's of AR tag visible           
##              cv2.circle(frame, (int(corner[0][0][0]),int(corner[0][0][1])), 5,(255, 255, 0),2)
##              cv2.circle(frame, (int(corner[0][1][0]),int(corner[0][1][1])), 5,(255, 0, 255),2)
##              cv2.circle(frame, (int(corner[0][2][0]),int(corner[0][2][1])), 5,(0, 255, 255),2)
##              cv2.circle(frame, (int(corner[0][3][0]),int(corner[0][3][1])), 5,(0, 255, 0),2)
    else:
        drive_val=[0,0,0] 
  
    frame = cv2.polylines(frame,[pts],isclosed,(0,0,255),2)     
    cv2.imshow('AR_LFR', frame)
                             
    if Ardu_stat==1 and port==1:
        arduino.write(bytes(drive_val)) #send data to bot
                             
    k=cv2.waitKey(1)
    if k==27:
        break
    elif k==ord('r'):
        try:
            pts=np.delete(pts,-1,axis=0)
        except:
            pass
    elif k==ord('l'):
        start=not(start)
        isclosed=not(isclosed)
    elif k==ord('q'):
        break

    c2 = cv2.getTickCount()
    #print(cv2.getTickFrequency()/(c2 - c1)) #if you want to see the FPS printed in console
    
    

if Ardu_stat==1 and port==1:	
    arduino.write(bytes([0,0,0]))
    arduino.close()
cv2.destroyAllWindows()
