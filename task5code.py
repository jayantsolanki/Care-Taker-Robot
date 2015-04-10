# -*- coding: cp1252 -*-
#########################Happy Holi to Eyantra Teams and Organisers############################
'''
*                                         Task 5,  (e-Yantra 2014), Main File
*                  ================================
*  Team Id: 		eYRC+#2447
*  Author List: 		Jayant Solanki, Uttam Kumar Gupta
*                                         Department of Electronics  & Communications, University of Allahabad.
*  Filename: 		task5code.py
*  Date:                               March 4, 2015
*  Theme: 		<eYRC-Plus> CareTaker Robot
*  Functions: 		execute(route_length,route_path), play(img,frame)
*  Global Variables:	grid_line_x,grid_line_y,width,height,grid_map[][],MIN,MAX,cap,bot_center_x_pixels,bot_center_y_pixels
*  Dependent library files:     numpy, cv2, time, imglib, motion, math, serial
*  
*  Software released under Creative Commons CC BY-NC-SA
*
*  For legal information refer to:
*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
*
*  This software is made available on an “AS IS WHERE IS BASIS”. 
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using 
*  ICT(NMEICT)
*
**************************************************************************
*     
*
*
'''
import numpy
import cv2
import serial #library function for accessing xbee module
import math
from time import sleep
from imglib import *
from motion import *
#image is diveded into 12x12 grid cells
grid_line_x = 13 #number of horizontal lines
grid_line_y = 13 #number of vertical lines
width=480/(grid_line_x-1) #grid cell width
height=540/(grid_line_y-1) #grid cell height
grid_map = [ [ 0 for i in range(grid_line_y-1) ] for j in range(grid_line_x-1) ] # zeros filled grid map is generated , which of 12x12, zero means no obstacle
#initialize web camera
cap = cv2.VideoCapture(1)

MIN= numpy.array([122,140,100],numpy.uint8) #dark blue colored mask
MAX= numpy.array([130,255,255],numpy.uint8)
#################
'''
* Function Name:	execute
* Input:		route_length -> integer which stores the length of sub-path calculated for the bot to traverse in the arena
*                           route_path-> array list to store the sub-path coodinates of the shortest path calculated for the bot to traverse
* Output:		NONE
* Logic:		This function accepts route_path and route_length from the play() function and then constantly monitors the position and orientation of the
*                           centre of the bot, and one by one using while loop directs the bot to move over the grid coordinates stored in the route path.
*                           Bot has two markers, dark blue colored. One placed at the central axis of the bot and other near the top of the bot
*                           Position of the bot is accessed by identifying both markers using contour detection.
*                           Orientation of the bot is detected using the slope the slope contructed using the centroids of both markers.
*                           Two slopes are calculated, one using both markers and other using central marker and coordinates of grid cell next to bot.
*                       
* Example Call:	execute(6,[(5,6), (5,7), (6,8), (7,8), (8,9), (9,9)]) #[(5,6), (5,7), (6,8), (7,8), (8,9), (9,9)] is a demo route_path for bot to follow
*
'''
def execute(route_length,route_path):
        #at every call of the execute function, stepper resets to 0
        stepper=0 #stepper: basically access the position of the immediate grid cell stored in the route_path, like an index to an array
        while(1): #beginning of the path processing list
                #capture shot of arena from the webcam
                ret, frame = cap.read()#frame stores the captured shot
                img=imgclip(frame) #imgclip function is called to return the clipped part of arena displayed under the black rectangle and is sotred in img variable
                ############ processing starts after clipping
                hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) #brg image is converted into hsv format image and is stored inhsv variable
                #image masking is done to identify the bot markers. Masked image is stored in bmask variable
                bmask = cv2.inRange(hsv, MIN,MAX)
                #bret,bthresh = cv2.threshold(bmask,127,255,1)
                #cv2.imshow('binary',mask)
                #cv2.imwrite("wall.jpg",mask)
                #process of identifying contours of those markers on the bot
                botcontours, bh = cv2.findContours(bmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                #contours identified are then sorted according to their areas in descending order and their numbers are limited to 2
                botcontours=sorted(botcontours, key = cv2.contourArea, reverse = True)[:2] ##botcontours: stores the contours detected
                cv2.drawContours(img,botcontours,-1,(255,255,0),2)
                #ccoor method is called from the imglib file to return pixel coordinates of the centroid of central marker
                bot_center_x_pixels,bot_center_y_pixels=ccoor(botcontours[0]) #both variables store the x and y coordinate pixels of bot's centre
                #centroid is displayed on the screencast for monitoring purpose
                cv2.circle(img,(bot_center_x_pixels,bot_center_y_pixels), 5, (0,0,255), -1)
                #print bot_center_x_pixels,bot_center_y_pixels
                ##ccoor method is called from the imglib file to return pixel coordinates of the centroid of other marker
                bot_other_x_pixels,bot_other_y_pixels=ccoor(botcontours[1]) #both variables store the x and y coordinate pixels
                cv2.circle(img,(bot_other_x_pixels,bot_other_y_pixels), 5, (0,100,100), -1)
                #getcoor method is called from motion file to convert pixel coordinates of central marker into grid coordinates
                bot_grid_x,bot_grid_y=getcoor(bot_center_x_pixels,bot_center_y_pixels,height,width)#both variable stores the grid coordinates
                #if condition will return true till stepper variable has reached the last element of the route_path
                if stepper!=route_length:
                        ser.write("9")#just for monitoring purpose, sounds off buzzer when bot starts to move to next grid cell
                        #gridtopixel method is called from motion file to convert grid cell's centre coordinates into their respective pixel coordinates
                        #grid cell's coordinates are accessed using route_path[stepper].x and route_path[stepper].y
                        grid_cell_X,grid_cell_Y=gridtopixel(route_path[stepper].x,route_path[stepper].y,width,height)#both variables stores grid cell's centre coordinates
                        cv2.circle(img,(grid_cell_Y,grid_cell_X), 5, (255,100,100), -1)#incoming cell's centre is displayed on screencast for monitoring purpose
                        #slope of the line joining bot's centre and incoming grid cell's centre is calculated
                        slope_bot2cell= getslope(bot_center_x_pixels,bot_center_y_pixels,grid_cell_Y,grid_cell_X)
                        #dis function is called from motion file to calculated distance between two points
                        distance_centre2cell=dis(bot_center_x_pixels,bot_center_y_pixels,grid_cell_Y,grid_cell_X)#distance between bot's center and next cell's pixel coordinates is stored
                        #slope of the line joining bot's central marker and other marker is calculated
                        slope_botmarkers= getslope(bot_center_x_pixels,bot_center_y_pixels,bot_other_x_pixels,bot_other_y_pixels)
                        distance_other2cell=dis(bot_other_x_pixels,bot_other_y_pixels,grid_cell_Y,grid_cell_X)#distance between bot's other point and next path coordinates
                        mid_x=(bot_center_x_pixels+bot_other_x_pixels)/2#mid point of bot center and other point
                        mid_y=(bot_center_y_pixels+bot_other_y_pixels)/2#mid point of bot center and other point
                        bot_grid_x,bot_grid_y=getcoor(mid_x,mid_y,height,width) #modified bot center
                        #important part of the code, responsible for initiating motion control of the bot.
                        #orientmove is called from the motion file and is responsible for turning the bot  and other motion control
                        #it accepts both slopes, bot central marker grid coordinates, route_path incoming cell coordinates and both distances variables
                        #function returns true if the central marker grid coordinates are equla to incoming grid coordinates, then the stepper variable is incremented by one
                        if orientmove(slope_bot2cell,slope_botmarkers,bot_grid_x+1,bot_grid_y+1,route_path[stepper].y+1,route_path[stepper].x+1,distance_centre2cell,distance_other2cell)==1: #bot reaches next coor
                                      stepper=stepper+1
                                      ser.write("7") #sounds the buzzer if bot reaches incoming grid cell
                                      #z,c=gridtopixel(route_path[stepper].x,route_path[stepper].y,width,height)
          
                else:
                        ser.write("5")#on reaching final grid cell, bot stops  and while loop breaks
                        #ser.write("9")
                        break
        ##########################
                        
                #cv2.imshow('maskcontour',bmask)
                #processed frame is shown as screencast, for monitoring purpose
                cv2.imshow('ori',img)
                if cv2.waitKey(1) == 27:  ## 27 - ASCII for escape key, use excape key to quit current processing
                        break
        ##############
##################################
#idetifying markers and returning their array coordinates
'''
* Function Name:	play
* Input:		img -> image taken from webcam
*                           frame-> image taken from webcam
* Output:		NONE
* Logic:		This functions first stores the coordinates of provisions, then detects the coordinates of tables(demand) and calculates path accordinagly then calls the execute function
*                           for executing the bot motion
*                       
* Example Call:	play(img,frame)
*
'''
def play(img,frame):
        marker = [ [ 0 for i in range(3) ] for j in range(8) ]
        count=0
        start=GridPoint(bot_grid_y,bot_grid_x)#bot start coordinates
        a1=0
        a2=0
        b1=0
        b2=0
        y1=0
        y2=0
        cR=0
        cB=0
        cY=0
        rx=0 # stores coordinates of red provision
        ry=0# stores coordinates of red provision
        bx=0# stores coordinates of blue provision
        by=0# stores coordinates of blue provision
        yx=0# stores coordinates of yellow provision
        yy=0# stores coordinates of yellow provision
        single=0
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        #Starting with red markers
        MIN= numpy.array([0,100,100],numpy.uint8)
        MAX= numpy.array([30,255,255],numpy.uint8)
        Rcontours=findcon(hsv,MIN,MAX)
        Rcontours=sorted(Rcontours, key = cv2.contourArea, reverse = True)[:5]
        Rcontours,length=areacon(Rcontours,1500,900)
        Rcontours=sorted(Rcontours, key = cv2.contourArea, reverse = True)[:length]
        cR=length-1
        rx,ry=provisions(Rcontours)
        cv2.drawContours(img,Rcontours,-1,(255,0,0),3)
        #print len(Rcontours)
        
        # blue markers
        MIN= numpy.array([95,100,100],numpy.uint8)
        MAX= numpy.array([130,255,255],numpy.uint8)
        Bcontours=findcon(hsv,MIN,MAX)
        Bcontours=sorted(Bcontours, key = cv2.contourArea, reverse = True)[:5]
        Bcontours,length=areacon(Bcontours,1500,600)
        Bcontours=sorted(Bcontours, key = cv2.contourArea, reverse = True)[:length]
        cB=length-1
        #print "Blue markers",length.
        bx,by=provisions(Bcontours)#getting provision grid coordinates
        cv2.drawContours(img,Bcontours,-1,(255,0,0),3)
        #yellow markers
        
        MIN= numpy.array([35,60,100],numpy.uint8)
        MAX= numpy.array([60,255,255],numpy.uint8)
        Ycontours=findcon(hsv,MIN,MAX)
        Ycontours=sorted(Ycontours, key = cv2.contourArea, reverse = True)[:5]
        #cv2.drawContours(img,Ycontours,-1,(255,0,0),3)
        Ycontours,length=areacon(Ycontours,1500,600)
        Ycontours=sorted(Ycontours, key = cv2.contourArea, reverse = True)[:length]
        cY=length-1
        yx,yy=provisions(Ycontours)
        cv2.drawContours(img,Ycontours,-1,(255,0,0),3)
        cv2.imshow("Rcon",img)
        #####returning coordinates of marker with maximum demands
        
        if len(Rcontours)>=len(Bcontours):
                if(len(Rcontours)>=len(Ycontours)):
                        ##code here for R
                        stop=GridPoint(ry,rx)
                        length,route=solve(start,stop,frame)
                        execute(length,route)#goes for red provision
                        ser.write("r")
                        start=stop
                        if(cY==cB==cR):
                                stop=GridPoint(yy,yx)#going for yellow provision
                                length,route=solve(start,stop,frame)
                                execute(length,route)
                                start=stop
                                ser.write("Y")
                                single=1
                        else:
                                ser.write("R")
                        count=0
                        for i in range(len(Rcontours)):
                                cx,cy=ccoor(Rcontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        a1,a2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(a2+1,a1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("o")
                                        if(count==1):
                                                ser.write("O")
                                        start=stop
                                        count=count+1
                                        if(count==2 and count!=cR): #if three red demands
                                                count=0
                                                stop=GridPoint(ry,rx) #again goes for red table if demand is three
                                                length,route=solve(start,stop,frame)
                                                execute(length,route)
                                                ser.write("r")
                                                start=stop
                                        
                        #code here for yellow
                        if(cY>0 and single==0):
                                stop=GridPoint(yy,yx)
                                length,route=solve(start,stop,frame)
                                execute(length,route)#goes to yellow provision
                                ser.write("Y")
                                start=stop
                                
                        for i in range(len(Ycontours)):
                                cx,cy=ccoor(Ycontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        y1,y2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(y2+1,y1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("O")
                                        start=stop
                                        count=count+1
                                        
                        #code for blue
                        if(cB>0):
                                stop=GridPoint(by,bx)
                                length,route=solve(start,stop,frame)
                                execute(length,route) #going for blue table
                                ser.write("b")
                                start=stop
                                
                        for i in range(len(Bcontours)):
                                cx,cy=ccoor(Bcontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        b1,b2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(b2+1,b1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("o")
                                        start=stop
                                        count=count+1
                else:
                        #code here for yellow
                        stop=GridPoint(yy,yx)
                        length,route=solve(start,stop,frame)
                        execute(length,route)#goes for yellow provision for provisions>1
                        ser.write("Y")
                        ser.write("y")
                        start=stop
                        count=0
                        for i in range(len(Ycontours)):
                                cx,cy=ccoor(Ycontours[i])
                                if cx > 270:#detecting provision marker
                                        y1,y2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(y2+1,y1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("O")
                                        if(count==1):
                                                ser.write("o")
                                        start=stop
                                        count=count+1
                                        if(count==2 and count!=cY):#for three yellow demands
                                                count=0
                                                stop=GridPoint(yy,yx)
                                                length,route=solve(start,stop,frame)
                                                execute(length,route)
                                                ser.write("Y")
                                                start=stop
                                                
                                        
                        #code for blue
                        print cB
                        if(cB>0):
                                #bx,by=getcoor(cx,cy,height,width)
                                stop=GridPoint(by,bx)
                                length,route=solve(start,stop,frame)
                                execute(length,route) #go for blue provision 
                                ser.write("b")
                                start=stop
                                
                        for i in range(len(Bcontours)):
                                cx,cy=ccoor(Bcontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        b1,b2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(b2+1,b1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)#going for blue table
                                        ser.write("o")
                                        start=stop
                                        count=count+1
                                        
                        #code for R
                        if(cR>0):
                                #rx,ry=getcoor(cx,cy,height,width)
                                stop=GridPoint(ry,rx)
                                length,route=solve(start,stop,frame)
                                execute(length,route)
                                ser.write("r")
                                start=stop
                                
                        for i in range(len(Rcontours)):
                                cx,cy=ccoor(Rcontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        a1,a2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(a2+1,a1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("o")
                                        start=stop
                                        count=count+1
                                        
                        
        else:
                if(len(Bcontours)>=len(Ycontours)):
                        stop=GridPoint(by,bx)
                        length,route=solve(start,stop,frame)
                        execute(length,route)#goes for blue provision, provisions>1
                        ser.write("b")
                        ser.write("B")
                        start=stop
                        count=0
                        for i in range(len(Bcontours)):
                                cx,cy=ccoor(Bcontours[i])
                                if cx > 270:#detecting provision marker
                                        b1,b2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(b2+1,b1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("o")
                                        if(count==1):
                                                ser.write("O")
                                        start=stop
                                        count=count+1
                                        if(count==2 and count!=cB): #if three blue demands
                                                count=0
                                                stop=GridPoint(by,bx)
                                                length,route=solve(start,stop,frame)
                                                execute(length,route)
                                                ser.write("b")
                                                start=stop
                        #code for R
                        if(cR>0):
                                stop=GridPoint(ry,rx)
                                length,route=solve(start,stop,frame)
                                execute(length,route)#going for red provision
                                ser.write("r")
                                start=stop
                        for i in range(len(Rcontours)):
                                cx,cy=ccoor(Rcontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        a1,a2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(a2+1,a1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)#going for red table
                                        ser.write("o")
                                        start=stop
                                        count=count+1
                        #code here for yellow
                        if(cY>0):
                                stop=GridPoint(yy,yx)
                                length,route=solve(start,stop,frame)
                                execute(length,route) #going for yellow provision
                                ser.write("Y")
                                start=stop
                        for i in range(len(Ycontours)):
                                cx,cy=ccoor(Ycontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        y1,y2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(y2+1,y1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)#going for yellow table
                                        ser.write("O")
                                        start=stop
                                        count=count+1
                                        
                else:
                        #code for Y
                        stop=GridPoint(yy,yx)
                        length,route=solve(start,stop,frame)
                        execute(length,route)#goes for yellow provision
                        ser.write("Y")
                        start=stop
                        count=0
                        for i in range(len(Ycontours)):
                                cx,cy=ccoor(Ycontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        y1,y2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(y2+1,y1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("O")
                                        if(count==1):
                                                ser.write("o")
                                        start=stop
                                        count=count+1
                                        if(count==2 and count!=cY): #for three yellow provisions
                                                count=0
                                                stop=GridPoint(yy,yx)
                                                length,route=solve(start,stop,frame)
                                                execute(length,route)
                                                ser.write("Y")
                                                start=stop
                        #code for R
                        if(cR>0):
                                stop=GridPoint(ry,rx)
                                length,route=solve(start,stop,frame)
                                execute(length,route)
                                ser.write("r")
                                start=stop
                        for i in range(len(Rcontours)):
                                cx,cy=ccoor(Rcontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        a1,a2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(a2+1,a1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("o")
                                        start=stop
                                        count=count+1
                                        
                        #code for B
                        if(cB>0):
                                stop=GridPoint(by,bx)
                                length,route=solve(start,stop,frame)
                                execute(length,route) #GO FOR BLUE PROVISION
                                ser.write("b")
                                start=stop
                                
                        for i in range(len(Bcontours)):
                                cx,cy=ccoor(Bcontours[i])
                                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                                if cx > 270:#detecting provision marker
                                        b1,b2=getcoor(cx,cy,height,width)
                                        stop=GridPoint(b2+1,b1)
                                        length,route=solve(start,stop,frame)
                                        execute(length,route)
                                        ser.write("o")
                                        start=stop
                                        count=count+1
                                        
        sleep(6)
        ser.write("9")#buzzer off
        
###########################################


#cap.set(11,40) #set brightness
'''
* Function Name:	Main program
* Input:		None
* Output:		None
* Logic:		Captures single frame from webcam. Frame is then processed for obstacle detection.
*                       
* Example Call:	Called automatically by the operating system
*
'''
#read shot from webcam
ret, img = cap.read() #img:stores the shot
#clip the part of arena inside the black lined rectangle
#imgclip function is called from imglib file to clip the arena
frame=imgclip(img) #frame: stores the clipped image
shot=frame #duplicating
hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)#hsv: stores the hsv format of frame
#obstacle function is called from imglib file to dilate the obstacle according to fitting criteria. See obstacle function for more details
dilation=obstacle(hsv)#dilation: stores the dilated image returned from obstacle function
#markobstacle function is called from imglib file to store the coordinates of obstacles in the grid_map array
grid_map,obb=markobstacle(dilation,frame,grid_line_x,grid_line_y)#grid_map: stores grid map of the clipped image marked with obstacle. obb: stores image marked with obstacles
#cv2.imwrite("dilation.jpg",dilation)
#cv2.imwrite("obstacles.jpg",obb)
#image masking is done to identify the bot markers. Masked image is stored in bmask variable
bmask = cv2.inRange(hsv, MIN,MAX) #bmask:stores the masked image
#process of identifying contour of central marker on the bot
bcontours, bh = cv2.findContours(bmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
bcontours=sorted(bcontours, key = cv2.contourArea, reverse = True)[:2] ##bcontours: stores the contours found
#ccoor method is called from the imglib file to return pixel coordinates of the centroid of central marker
bot_center_x_pixels,bot_center_y_pixels=ccoor(bcontours[0]) #both variables store the x and y coordinate pixels of the bot's centre
cv2.circle(frame,(bot_center_x_pixels,bot_center_y_pixels), 5, (0,0,255), -1)#displaying bot's centre on screen
################
#getcoor method is called from motion file to convert pixel coordinates of central marker into grid coordinates
bot_grid_x,bot_grid_y=getcoor(bot_center_x_pixels,bot_center_y_pixels,height,width)#both variable stores the grid coordinates of the bot's centre
start=play(shot,frame)#play function is called start processing the paths for provisions and demands
######################
cv2.destroyAllWindows()
