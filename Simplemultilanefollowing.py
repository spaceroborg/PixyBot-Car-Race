#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 19 13:59:12 2023

@author: juliesarrazin
"""

## library for obstacle avoidance and lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2020

from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
from turningPad import turningPad
from PIDcontroller import PID_controller
import math
from turningPad import turningPad


# obstacleAvoidance class: has a bunch of convinient functions for navigation
#
# input arguments
#   bot                         - (optional) pixyBot object
#   cam                         - (optional) pixyCam object
#
# attributes - feel free to add more!
#   bot                         - pixyBot object
#   cam                         - pixyCam object
#   IDs                         - signature ID number for various colour blocks
#   frameTimes                  - list of frameTimes for blockSize and blockAngle
#   blockSize                   - list of angular size of queried block over frameTimes in ([deg])
#   blockAngle                  - list of angular error of queried block over frameTimes in ([deg])
#
# methods
#   drive                       - differential drive function that takes bias for the right wheel speed
#   getBlockParams              - get and store the size and anglular error of a selected block
#   visTrack                    - move camera to point at selected block
#   stopAtStationaryObstacles   - move bot along a lane until it encounters an obstacle
#   avoidStationaryObstacles    - move bot along a lane and move to avoid stationary obstacles
class obstacleAvoidance(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam(), turn = turningPad()):
        self.bot = bot

        self.bot = bot
        self.cam = cam
        self.turn = turn

        self.centerLineID   = 1
        self.leftLineID     = 2
        self.rightLineID    = 3
        self.obstacleID     = 4
        
        self.lanectrl = PID_controller(0.02, 0, 0)
        

        self.obstacleW      = 4.5 #cm

        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]

    # output            - none
    # drive             - desired general bot speed (-1~1)
    # bias              - ratio of drive speed that is used to turn right (-1~1)
    def drive(self, drive, bias): # Differential drive function
        if bias > 1:
            bias = 1
        if bias < -1:
            bias = -1

        maxDrive = 1 # set safety limit for the motors

        totalDrive = drive * maxDrive # the throttle of the car
        diffDrive = bias * totalDrive # set how much throttle goes to steering
        straightDrive = totalDrive - abs(diffDrive) # the rest for driving forward (or backward)

        lDrive = straightDrive + diffDrive
        rDrive = straightDrive - diffDrive
        self.bot.setMotorSpeeds(lDrive, rDrive)

    # output            - -1 if error
    # blockIdx          - index of block in block list that you want to save parameters for
    def getBlockParams(self, blockIdx):
        if (self.cam.newCount-1) < blockIdx or blockIdx < 0: # do nothing when block doesn't exist
            return -1
        else:
            pixelSize = self.cam.newBlocks[blockIdx].m_width;
            angleSize = pixelSize/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x -  self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular error of block relative to front

            # save params
            self.blockSize.append(angleSize)
            self.blockAngle.append(angleError)
            self.frameTimes.append(time())

            # remove oldest params
            self.blockSize.pop(0)
            self.blockAngle.pop(0)
            self.frameTimes.pop(0)

            return pixelSize, angleError

    # output            - tracking error in ([deg]) and new camera angle in ([deg]) (tuple), or -1 if error
    # blockIdx          - index of block in block list that you want to track with the camera
    def visTrack(self, blockIdx): # Get pixycam to rotate to track an object
        if blockIdx < 0: # do nothing when block doesn't exist
            self.bot.setServoPosition(0)
            return 0, 0
        else:
            pixelError =  self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX # error in pixels
            visAngularError = -(pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV) # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition



    # output            - none
    # speed             - general speed of bot
    def avoidStationaryObstacles(self, speed):

        self.bot.setServoPosition(0)
        self.drive(0, 0)

        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID)
            
                
         #sensor detection 
            if self.cam.isInView(self.centerLineID) >= 0:
                self.getBlockParams(self.cam.isInView(self.centerLineID))
                centerblockAngle = -self.blockAngle[-1]
                visAngularError, newServoPosition = self.visTrack(centerLineBlock)
                print("center line angle") 
                print(newServoPosition)
                print(self.blockAngle)
                print(centerblockAngle)
             
             
            elif self.cam.isInView(self.leftLineID) >= 0:
                self.getBlockParams(self.cam.isInView(self.leftLineID))
                leftblockAngle = -self.blockAngle[-1]
                leftServoPosition = self.bot.servo.lastPosition[-1]
                #visAngularError, newServoPosition = self.visTrack(leftLineBlock)
                leftServoPosition = newServoPosition
                print("left line angle") 
                print(leftServoPosition)
                print(leftblockAngle)
               
                
            elif self.cam.isInView(self.rightLineID) >= 0:
                self.getBlockParams(self.cam.isInView(self.rightLineID))
                rightServoPosition = self.bot.servo.lastPosition[-1] 
                rightblockAngle = -self.blockAngle[-1] 
                print("right line angle") 
                print(self.blockAngle)
                print(rightblockAngle)
                
                
                #split this one into two if segments? 
     
        #control part of the algorithm 
        #to steer the robot according to blocks detected 
            if centerLineBlock>=0: 
                print("follow center line")
                error = -centerblockAngle - newServoPosition
                print(newServoPosition)
                print(error)
                bias = self.lanectrl.update(error)
                self.drive(speed, bias)

                if abs(newServoPosition) > 50:
                     print("RECALIBRATING")
                     self.bot.setServoPosition(0)
             
            if leftLineBlock>=0 and rightLineBlock>=0 and centerLineBlock<0: 
                print("follow left line")
                print("both lines detected")
                centerlineangle=0 #can amend that value during tuning - useful when trying to keep the robot within the tracks 
                newServoPosition = self.bot.setServoPosition(centerlineangle)
                trackblockangle = (rightblockAngle + leftblockAngle)/2
                print("average block angle")
                print(trackblockangle)
                error = -trackblockangle - newServoPosition
                print(error)
                bias = self.lanectrl.update(error)
                self.drive(speed, bias)
         
            else:
                self.drive(0,0)
           
            #other possibility when looses all the blocks 
            #is to use the last saved block angle position to determine in which direction it has to turn 
            # 
         #     if visAngularError>=0 :
            #       direction= -1
            #  elif visAngularError<0:
            #       direction=1 
                
        #can then use that to either turn the pixybot camera neck and scan in the wanted direction 
            # scanningangle= direction*50
            #newServoPosition = self.bot.setServoPosition(scanningangle)
        #or can use the turning pad function to turn the robot in case the first option 
            # servoCorrection = 0
            # r = pixyBot(servoCorrection)
            # tp = turningPad(r)
            # velocity=0.5
            # turnrate= direction*velocity
            # turntime= visAngularError/turnrate
            # tp.symmetricTurn(turnrate,turntime)  #tune for time how can i make time so that it's until it sees something -- do i put a while loop? but then won't leave 

        return