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
*  Functions: 		areacon(contours,area,sub), imgclip(frame), obstacle(hsv), markobstacle(obb,imgg,m,n), grid_draw(img,mm,nn), solve(start,finish,img), ccoor(contour)
*                                         provisions(contours)
*  Global Variables:	grid_line_x,grid_line_y, width, height, grid_map
*  Dependent library files:     math, serial, cv2, numpy, heapq, motion
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
import heapq
from motion import *
#image is diveded into 12x12 grid cells
grid_line_x = 13 #number of horizontal lines
grid_line_y = 13 #number of vertical lines
width=480/(grid_line_x-1) #grid cell width
height=540/(grid_line_y-1) #grid cell height
grid_map = [ [ 0 for i in range(grid_line_x-1) ] for j in range(grid_line_y-1) ]
'''
* Function Name:	areacon
* Input:		contours: stores contours detected
*                           area: integer, which stores stated area
*                           sub: integer,  which stores variation in area
*                           y2: integer which stores y coordinate of second point
* Output:		returns contours with area under the given limits and new contour size with unwanted contours left out
* Logic:		checks which area falls under the given limit and return those only
*                       
* Example Call:	areacon(contours, 1500,900) 
*
'''
def areacon(contours,area,sub):
        count=0#count:stores the integer value, and is inceremented if area under limit is found
        #con=0
        for i in range(len(contours)):#looking into contours
                ar = cv2.contourArea(contours[i])
                #print ar,area
                if ar>area-sub and ar<area+sub:#detecting areas which are under limits
                        contours[count]=contours[i]
                        count=count+1
                        #print count
        return contours,count        
                        
                        
                        
        

########################################
##############
# Image clipper
'''
* Function Name:	imgclip
* Input:		frame: stores frame taken from webcam
* Output:		returns clipped frame
* Logic:		searches for the biggest rectangle(quadrilateral) with black sides and return the image under it
*                       
* Example Call:	imgclip(frame)
*
'''
def imgclip(frame):
        #converting bgr image into gray format
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#gray:stores new image
        lower = numpy.array([0, 0, 0]) #black color mask
        upper = numpy.array([120, 120, 120])
        mask = cv2.inRange(frame, lower, upper)#mask:stores masked image

        ret,thresh1 = cv2.threshold(mask,127,255,cv2.THRESH_BINARY)#thresholding the image
        contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#detecting and storing contours
        #cv2.drawContours(frame,contours,-1,(0,255,0),3)
        biggest = 0
        max_area = 0
        min_size = thresh1.size/4
        index1 = 0
        for i in contours: #searching for biggest arean falling under contours
                area = cv2.contourArea(i)
                if area > 10000:
                    peri = cv2.arcLength(i,True)
                if area > max_area: 
                    biggest = index1
                    max_area = area
                index1 = index1 + 1
        approx = cv2.approxPolyDP(contours[biggest],0.05*peri,True)#detecting quadrilateral
        #drawing the biggest polyline
        #cv2.polylines(frame, [approx], True, (0,255,0), 3)
        #defining 4 sides of quadrilateral
        x1 = approx[0][0][0]
        y1 = approx[0][0][1]
        x2 = approx[1][0][0]
        y2 = approx[1][0][1]
        x3 = approx[3][0][0]
        y3 = approx[3][0][1]
        x4 = approx[2][0][0]
        y4 = approx[2][0][1]
        #print x1,y1,x2,y2,x3,y3,x4,y4
        

        #points remapped from source image from camera
        #to cropped image try to match x1, y1,.... to the respective near values
        pts1 = numpy.float32([[x1,y1],[x2,y2],[x3,y3],[x4,y4]]) 
        pts2 = numpy.float32([[0,0],[0,480],[540,0],[540,480]]) #strecthing the image to given image dimensions
        persM = cv2.getPerspectiveTransform(pts1,pts2)
        img = cv2.warpPerspective(frame,persM,(540,480))
        return img
        ###clipping ends
############
############
'''
*Function Name:	obstacle
*Input:       hsv image of initial instant
*Output:	provide dilated image of obstacle or walls
*Logic:	did closing operation on masked walls followed dilated to give appropriate width for obstacle avoidance
*
'''
############################################
def obstacle(hsv):
    lower = numpy.array([65 ,110, 50],numpy.uint8)                      # Lower hsv mask value of walls 
    upper = numpy.array([100, 255, 255],numpy.uint8)                    # upper hsv mask value of walls
    mask = cv2.inRange(hsv,lower, upper)                                # walls masked
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:14] #contour of walls
    contours,length=areacon(contours,5000,4100)                           #discard other than walls
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:length] #walls contours
    cv2.fillPoly(mask,contours, (255,255,255))                          #filling the contour of walls
    kernel = numpy.ones((75,60),numpy.uint8)                            #for morphological operation(erosion and dilation) initialized array
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)           #done closing (erosion followed by dilation)
    dilation = cv2.dilate(closing,kernel,iterations = 1)                #dilated output image from closing operation
    return dilation

#################################
#################################
'''
*Function Name:	markobstacle
* Input:                 dilated image of obstacles of initial instant
* Output:	              grid_map with  obstacles marked
*Logic:                  threshold the center of grids to check for obstacles and mapping them into grid_map
*
'''
def markobstacle(obb,imgg,m,n):
        h,k,l=imgg.shape
        widm=h/(m-1)    #width of grid in x direction of image
        widn=k/(n-1)    #width of grid in y direction of image
        '''
        img-- a single test image as inumpyut argument
        route_length  -- returns the single integer specifying the route length
        '''
        global grid_map

        #cv2.imshow("walls in grid map",obb)
        for x in range(0,m-1):
                X=x*widm+(widm/2)
                for y in range(0,n-1):
                        Y=y*widn+(widn/2)
                        if obb[X,Y]>=250 or x==0 or x==m-2 or y==0 or y==n-2 :  #mark obstacles (from obb image) and boundaries in grid_map with having values 1 in obstacle image 
                                grid_map[x][y]=1
                                cv2.circle(imgg,(Y,X), 5, (0,50,200), -1)       #draw points in image for showing obstacles
                        continue
        #print grid_map
        return grid_map,imgg #return grid map
        

##############################################
##################
# grid draw
'''
* Function Name:	grid_draw
* Input:		img: frame taken from webcam
*                           mm:horizontal lines
*                           nn: vertical lines
* Output:		returns image with grid drawn on it
* Logic:		draws grid of given dimensions using iteration method
*                       
* Example Call:	grid_draw(img,13,13)
*
'''
def grid_draw(img,mm,nn): ##filename is image filename with full file path, n is grid of n lines

    #img=cv2.imread(filename) ##getting input image
    h,k,l=img.shape
    widm=h/(mm-1)
    widn=k/(nn-1)
    for x in range(0, mm): ##drawing lines
        X=x*widm
        cv2.line(img,(0,X),(k,X),(0,0,0), 2)#lines is red color, bgr format
    for y in range(0, nn): ##drawing lines
        Y=y*widn
        cv2.line(img,(Y,0),(Y,h),(0,0,0), 2)
    return (img)
###################
##############################
#solvegrid
'''
* Function Name:	solve
* Input:		start: start grid coordinates
*                           finish:end grid coordinates
*                           img: input images where path will be drawn
* Output:		returns route path and route length
* Logic:		finds shortest path using dijkstra's method on a gridmap marked with obstacles
*                       
* Example Call:	solve([2,2],[14,2],img)
*
'''
def solve(start,finish,img): #no heuristics used
    """Find the shortest path from START to FINISH."""
    
    heap=[]
    link = {} # parent node link
    g = {} # shortest path to a current node
    
    g[start] = 0 #initial distance to node start is 0
    
    link[start] = None #parent of start node is none
    
    
    heapq.heappush(heap, (0, start))
    
    while True:
        
        f, current = heapq.heappop(heap) ##taking current node from heap
        #print current
        if current == finish:
            name='Shortest Path, image#'
            i=int(100*numpy.random.rand())
            name=name+str(i)
            route=build_path(start, finish, link)
            ####Drawing path , just for pictorial representation######
            for i in range(1,len(route)):
                cv2.line(img,(route[i-1].y*n+(n/2),route[i-1].x*m+(m/2)),(route[i].y*n+(n/2),route[i].x*m+(m/2)),(232,162,0), 3)
            cv2.imshow('name',img)
            ############################
            return g[current], route[1:len(route)]
            
        
        moves = current.get_moves()
        cost = g[current]
        for mv in moves:
            #print mv.x,mv.y
            if grid_map[mv.x][mv.y]==1: #bypass obstacles
                continue
                #mv is the neighbour of current cell, in all maximum 6 neighbours will be there
            if  (mv not in g or g[mv] > cost + 1): #check if mv is already visited or if its cost is higher than available cost then update it
                g[mv] = cost + 1
                
                link[mv] = current #storing current node as parent to mv 
                heapq.heappush(heap, (g[mv], mv)) ##adding updated cost and visited node to heap
                #cv2.circle(orig_img,(mv.y*n+n/2,mv.x*m+m/2), 5, (255,144,0), -1)

###########################################################   
def build_path(start, finish, parent):
    
    #create path from start to finish

    x = finish ##back tracking the path from goal to start
    xs = [x]
    while x != start: #going back
        x = parent[x]
        xs.append(x)
    xs.reverse()
 
    return xs

###########################################################
class GridPoint(object):
    """Represent a position on a grid."""
    def __init__(self, x, y): #self referencing x and  y coordinates
        self.x = x
        self.y = y

    def __hash__(self): #returning hash value of the GridPoint object
        return hash((self.x, self.y))

    def __repr__(self):                         #returns values stored in current object, values are x and y coordinates
        return "(%d,%d)" % (self.y+1, self.x+1)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def get_moves(self): ##taking current node coordinates to find neighbours of it
        
        
        if self.x>=0 and self.x<=len(grid_map)-1 and self.y>=0 and self.y<=len(grid_map)-1:
            if self.x + 1<len(grid_map):
                yield GridPoint(self.x + 1, self.y)
            if self.y + 1<len(grid_map):  
                yield GridPoint(self.x, self.y + 1)
            if self.x - 1>=-1:
                yield GridPoint(self.x - 1, self.y)
            if self.y - 1>=-1:
                yield GridPoint(self.x, self.y - 1)
                #############################
               
            #if self.x + 1<len(grid_map) and self.y + 1<len(grid_map):
                #yield GridPoint(self.x + 1, self.y+1)
            if self.y + 1<len(grid_map) and  self.x - 1>-1:  
                yield GridPoint(self.x-1, self.y + 1)    
            #if self.x - 1>-1 and self.y - 1>-1:
                #yield GridPoint(self.x - 1, self.y-1)
            if self.y - 1>-1 and self.x + 1<len(grid_map):
                yield GridPoint(self.x+1, self.y - 1)
               
        

            
                
            
                
            

#############################################################
###########################################
# calculate contours from np array
'''
* Function Name:	findcon
* Input:		hsv: hsv formatted image
*                           MIN:masking lower limit
*                           MAX: masking upper limit
* Output:		returns contours with given mask
* Logic:		using masking and thresholding, given contours are returned
*                       
*
'''
def findcon(hsv,MIN,MAX):
        mask = cv2.inRange(hsv, MIN,MAX)#masking the image and storing it into mask variable
        ret,thresh = cv2.threshold(mask,127,255,1)#thresh:stores the thresholded image
        contours, h = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#stores contours found
        return contours
###########################################

###########################################
# calculate centroid coordinates of given contour
'''
* Function Name:	ccoor
* Input:		contour: a particular contour
* Output:		returns centroid coordinates of given contour
* Logic:		using Moments, coordinates are found
*                       
*
'''
def ccoor(contour):
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])#cx: stores the x coordinates of centroid
        cy = int(M['m01']/M['m00'])#cx: stores the y coordinates of centroid
        return cx,cy
###########################################
'''
* Function Name:	provisions
* Input:		contours: given contour array
* Output:		returns coordinates of provision found
* Logic:		if x coordinates lies before 270 then it is a provision
*                       
*
'''
def provisions(contours):
        for i in range(len(contours)):
                cx,cy=ccoor(contours[i])#getting centoird's coordinates
                #cv2.circle(orig_img,(cx,cy), 5, (255,255,0), -1)
                if cx < 270:#detecting provision marker
                        #count=count+1
                        px,py=getcoor(cx,cy,height,width)
                        return px,py
                         


