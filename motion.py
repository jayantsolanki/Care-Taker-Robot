# -*- coding: cp1252 -*-
#########################Happy Holi to Eyantra Teams and Organisers############################
'''
*                                         Task 5,  (e-Yantra 2014), Library File
*                  ================================
*  Team Id: 		eYRC+#2447
*  Author List: 		Jayant Solanki, Uttam Kumar Gupta
*                                         Department of Electronics  & Communications, University of Allahabad.
*  Filename: 		motion.py
*  Date:                               March 4, 2015
*  Theme: 		<eYRC-Plus> CareTaker Robot
*  Functions: 		getcoor(pixel_x,pixel_y,height,width), gridtopixel(grid_x,grid_y,height,width), dis(x1,y1,x2,y2), getslope(x1,y1,x2,y2),
*                                         orientmove(slope_bot2cell,slope_botmarkers,bot_grid_x,bot_grid_y,route_x,route_y,distance_centre2cell,distance_other2cell)
*  Global Variables:	grid_line_x,grid_line_y
*  Dependent library files:     math, serial
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
import math
import serial #serial file is imported for controlling xbee module
ser=serial.Serial(3) #COM4
grid_line_x = 13 #number of horizontal lines
grid_line_y = 13 #number of vertical lines
##########################
'''
* Function Name:	getcoor
* Input:		pixel_x: integer which stores pixel's x coordinates
*                           pixel_y: integer which stores pixel's y coordinates
*                           heigtht: integer, stores height of grid cell
*                           width: integer, stores width of grid cell
* Output:		returns grid cell's coordinates
* Logic:		Converts pixel coordinates into their specific grid coordinates under which those pixels lies.
*                       
* Example Call:	getcoor(233,245,30,33)
*
'''
def getcoor(pixel_x,pixel_y,height,width):
        '''
        cx=x/n#(int)(round(x/m))
        cy=y/n#(int)(round(y/n))
        return cx,cy
        '''
        #img=cv2.imread(filename) ##getting input image
        height_X=0# stores 0 for initial  pixel's x coordinate
        width_Y=0# stores 0 for initial  pixel's y coordinate
        for i in range(0, grid_line_x): ##drawing lines
                #step by step increases the pixel's x coordinates by adding width
                height_X=height_X+width
                width_Y=0
                for j in range(0, grid_line_y): ##drawing lines
                        #step by step increases the pixel's y coordinates by adding height
                        width_Y=width_Y+height
                        #print X,Y
                        if pixel_x<=height_X and pixel_y<=width_Y:# if pixel_x and pixel_y lies under height_X and width_Y, then grid cell coodinates is returned
                                return i,j #return respective grid coordinates
                                break
##########################
# converting grid coordinates into pixels
'''
* Function Name:	gridtopixel
* Input:		grid_x: integer which stores grid cell x coordinates
*                           grid_y: integer which stores grid cell y coordinates
*                           height: integer, stores height of grid cell
*                           width: integer, stores width of grid cell
* Output:		returns grid cell's pixel coordinates
* Logic:		converts grid cell's centre coordinates into pixel coodinates
*                       
* Example Call:	gridtopixel(3,3,30,33)
*
'''
def gridtopixel(grid_x,grid_y,height,width):
        #accessing centre of grid cell by multiplying grid coordinates with respective height and width and adding their halves
        pixel_x=grid_x*height+height/2 #pixel_x: stores the integer value of pixel's x coodinate
        pixel_y=grid_y*width+width/2 #pixel_y: stores the integer value of pixel's y coodinate
        return pixel_x,pixel_y
########################
##########################
# getting distance between two coordinate points
'''
* Function Name:	dis
* Input:		x1: integer which stores x coordinate of first point
*                           y1: integer which stores y coordinate of first point
*                           x2: integer which stores x coordinate of second point
*                           y2: integer which stores y coordinate of second point
* Output:		returns distance between both points
* Logic:		uses coordinate geometry for finding distance between two points
*                       
* Example Call:	dis(230,250,400,350)
*
'''
def dis(x1,y1,x2,y2):
        dist=math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))#square root function is called
        dist=int(dist)#dist:stores the integer value of the distance
        #print dist
        return dist
########################
#get slope 
'''
* Function Name:	getslope
* Input:		x1: integer which stores x coordinate of first point
*                           y1: integer which stores y coordinate of first point
*                           x2: integer which stores x coordinate of second point
*                           y2: integer which stores y coordinate of second point
* Output:		returns slope between both points
* Logic:		uses coordinate geometry for finding slope between two points, m=(y2-y1)/(x2-x1)
*                           Since I am considering the arena coordinates as 4th quadrant , so m is multiplied with -1
*                           tan(-theta)=-tan(theta)
* Example Call:	getslope(230,250,400,350)
*
'''
def getslope(x1,y1,x2,y2):
        m=0
        if x2-x1!=0:#checking if slope is not infinte
                m=-(float)(y2-y1)/(x2-x1)#using slope function of coordinate geometry
                return m
        else:
                return 50#m>50 for angle>88 degrees or (180-88), in case slope is approaching infinite
############################
# getting orientation of the bot and moving it
'''
* Function Name:	orientmove
* Input:		slope_bot2cell: float which stores slope of line crossing bot's centre and next grid cell's centre
*                           slope_botmarkers: float which stores slope of line crossing bot's centre and its' other marker
*                           bot_grid_x: integer which stores grid's x coordinate where bot is currently in
*                           bot_grid_y: integer which stores grid's y coordinate where bot is currently in
*                           route_x: integer which stores next grid cell's x coordinates
*                           route_y: integer which stores next grid cell's y coordinates
*                           distance_centre2cell: integer which stores distance between bot's centre and next grid cell's centre
*                           distance_other2cell: integer which stores distance between bot's centre and its' other marker
* Output:		returns 1 if bot reaches next grid cell's centre coordinates else 0
* Logic:		uses coordinate geometry for finding orientation.
* Example Call:	orientmove(0.2,0.3,2,2,3,3,33,20)
*
'''
def orientmove(slope_bot2cell,slope_botmarkers,bot_grid_x,bot_grid_y,route_x,route_y,distance_centre2cell,distance_other2cell):
        if bot_grid_x==route_x and bot_grid_y==route_y: #check if bot has reached next coordinate
                #ser.write("5")
                #print "Hello"
                #ser.write("7")
                return 1
        else:
                
                if slope_bot2cell*slope_botmarkers!=-1 :
                        theta=math.atan((slope_bot2cell-slope_botmarkers)/(1+slope_bot2cell*slope_botmarkers))
                        #print theta
                        if distance_other2cell>distance_centre2cell: #if other marker is farther from the grid cell's centre then move it closer with fast turns
                                 if theta<20: #20 for theta greater than 90 degrees, here theta is in radians
                                       ser.write("D")  #fast right turn
                                 else:
                                       ser.write("A")   #fast left turn
                        elif (theta<-0.18 or theta>0.18): #align the bot # if marker is closer to grid cell's centre, then try to align it with the cell's centre
                                #com=1
                                if theta<-0.18:
                                       ser.write("6")  #right turn
                                else:
                                       ser.write("4")   #left turn
                                #com = raw_input()
                                
                                #ser.write(com) #send command
                        
                        
                        else:#if theta approaches zero, it means bot is nearly alighned with the next grid cell and it can move forward
                               ser.write("8") #forward bot
                return 0
#############################
