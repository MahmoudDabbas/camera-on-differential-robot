#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'

cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
newx = 320
newy = 240

nx = 640
ny = 480

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'

def FindGreen(img):
	empty = np.zeros((img.shape[0],img.shape[1]),np.int)
	for j in range(0,img.shape[1]):
		for i in range(0,img.shape[0]):
			#print img[i,j]
 			if img[i,j][1]>100 and img[i,j][0]<20 and img[i,j][2]<20:
				empty[i,j]=1

	return empty
while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl", img)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
#        print 'Sim Time 2 = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    imgResized=cv2.resize(img,(256,192))
    modifiedIMG=FindGreen(imgResized) #return a matrix with ones in location of green pixels
    sumX=0 #to help locate the center of the green ball 
    sumY=0 #to help locate the center of the green ball
    centerX=0 #the location if the center of the green ball
    centerY=0 #the location if the center of the green ball
    counter=0 #to help locate the center of the green ball
    imgFinal=np.zeros((imgResized.shape[0],imgResized.shape[1],3),np.uint8) #an empty black image for final result on screen
    for j in range (0,modifiedIMG.shape[1]): #this loop to locate the green pixels (here ones) and register their locations
	for i in range (0,modifiedIMG.shape[0]):
		if modifiedIMG[i,j]==1:
			sumX=sumX+i
			sumY=sumY+j
			counter=counter+1 #update the counter
    if counter ==0:
	print "error" #print error to avoid deviding by zero
    else:
	centerX=sumX/counter #calculate the center location
	centerY=sumY/counter #calculate the center location
	print "centerX : ",centerX
	print "centerY : ",centerY
    	cv2.circle(imgFinal,(centerY,centerX),5,(0,0,255),4) #print a circle in the center location
    cv2.imshow("imgFinal",imgFinal) #print the final output
	
#    print img[0,0][0]
    #blank=np.zeros((400,200,3),np.uint8)
#    print blank
    #blank[:,0:0.5*200]=(0,0,255)
    #cv2.imshow("blank",blank)
#    print img
    ref.ref[0] = -.5 #determine the speed of the robot 
    ref.ref[1] = .5 #determine the speed of the robot 

#    print 'Sim Time = ', tim.sim[0]
    
    r.put(ref);

    # Sleeps
    time.sleep(.001)   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
