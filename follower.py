from pixyBot import pixyBot
from pixyCam import pixyCam
from PIDcontroller import PID_controller
import math
from time import time,sleep

# laneFollower class: has a bunch of convinient functions for navigation
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

class laneFollower(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
      
        self.bot = bot
        self.cam = cam
      
        self.centerLineID   = 1
        self.leftLineID     = 2
        self.rightLineID    = 3

        self.biasControl = PID_controller(0.015, 0, 0.000)  # 0.015 # 0.001
        self.blueControl = PID_controller(0.005, 0, 0.001)
        
        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]
        self.pixelSize = [float('nan') for i in range(nObservations)]
        self.position = [float('nan') for i in range(nObservations)]

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
            position = self.cam.newBlocks[blockIdx].m_x

            # save params
            self.pixelSize.append(pixelSize)
            self.position.append(position)
            self.blockSize.append(angleSize)
            self.blockAngle.append(angleError)
            self.frameTimes.append(time())

            # remove oldest params
            self.pixelSize.pop(0)
            self.blockSize.pop(0)
            self.blockAngle.pop(0)
            self.frameTimes.pop(0)
            self.position.pop(0)
            
    # output            - tracking error in ([deg]) and new camera angle in ([deg]) (tuple), or -1 if error
    # blockIdx          - index of block in block list that you want to track with the camera
    def visTrack(self, blockIdx): # Get pixycam to rotate to track an object
        if blockIdx < 0: # do nothing when block doesn't exist
            self.bot.setServoPosition(0)
            return -1
        else:
            pixelError =  self.cam.newBlocks[blockIdx].m_x - self.cam.pixyCenterX # error in pixels
            visAngularError = -(pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV) # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition        
    
    # output            - index of the largest block visible (0, 1, or 2)
    # centerLineID      - colour ID of center line
    # leftLineID        - colour ID of left line
    # rightLineID       - colour ID of right line
    def findLargestBlockIDX(self, centerLineID, leftLineID, rightLineID):

        if centerLineID == -1:
            centerLineID = 100
        if leftLineID == -1:
            leftLineID = 100
        if rightLineID == -1:
            rightLineID = 100

        largestBlockIDX = -1
        all_blocks = [centerLineID, leftLineID, rightLineID]  
        if all_blocks[all_blocks.index(min(all_blocks))] < 100:
            largestBlockIDX = all_blocks.index(min(all_blocks))
        return  largestBlockIDX
  
    # function to follow the lane 
    def follow(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop

        bias = 0
        
        # set offsets of the left and right lines
        left_offset = 23
        right_offset = -23
        
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            
            lines = [centerLineBlock, leftLineBlock, rightLineBlock]
                 
            # Find the index of the colour block which is the largest (out of red, blue (left), purple (right))
            largestBlockIDX = self.findlargestBlockIDX(centerLineBlock, leftLineBlock, rightLineBlock)

            if largestBlockIDX >= 0: # when largest block is visible                    
                
                # get parameters of the largest block
                self.getBlockParams(lines[largestBlockIDX])                    

                # Follow red if red is the largest block
                if largestBlockIDX == 0:
                    visAngularError, newServoPosition = self.visTrack(centerLineBlock)
                    error = -visAngularError-newServoPosition-5
                    bias = self.PID_control.update(error)
                    #print("following red")
                     
                # Follow purple (right) if purple (right) is the largest block
                elif largestBlockIDX == 2:
                    
                    error = self.blockAngle[-1] + right_offset - 15
                    bias = self.PID_control.update(error)
                    #print("following purple")
                    
                # Follow blue (left) if blue (left) is the largest block
                elif largestBlockIDX == 1:
                    
                    error = self.blockAngle[-1] + left_offset + 15
                    bias = self.PID_control.update(error)
                    #print("following blue")

                # For lane following when no red line is visible
                else:
                    print("Red is not the largest block")
                    #turn camera so that it catches blue/purple line and does not rotate to find red
                    setServo = self.bot.setServoPosition(20)
                        
                    self.getBlockParams(leftLineBlock)
                    left_offset = self.position[-1]
                    #print("left line position")
                        
                    self.getBlockParams(rightLineBlock)
                    right_offset = self.position[-1]
                    #print("right line position")
                        
                    # use average of the left line and right line to get desired bot position 
                    position = (eft_offset - right_offset)/2 + 10
                    #print("getting position average")
                    bias = self.PID_control.update(position)
                    #print("using position average for steering")

                    #turn camera away from purple line to encourage the bot to look towards red line
                    setServo = self.bot.setServoPosition(-10) 
                     
            else:
                speed = 0
                bias = 0                    
                        
            self.drive(speed, bias)
