from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
from PIDcontroller import PID_controller
import math


class robotRace(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
        self.bot = bot
      
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
        
      
    def largest_block(self,center_id, left_id, right_id):
        # If there is no block of either color, set that id really high - e.g 50
        if center_id == -1:
            center_id = 50
        if left_id == -1:
            left_id = 50
        if right_id == -1:
            right_id = 50
        
        # group all the marker ids together
        array = [center_id, left_id, right_id]  
        
        # set initial id to -1 (can be used later to determine if the camera sees anything or not)
        largest_block_id = -1
        
        # If color markers are recognised (i.e. min number in array is smaller than 50)        
        if array[array.index(min(array))] < 50:
            # find the id of the largest block
            largest_block_id = array.index(min(array))
        
        # return the id of the largest block: 0-center, 1-left, 2-right, -1- no marker recognised
        return  largest_block_id
       
    def race(self, speed):
        # Initialise the important variables
        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop

        lineSteering = 0
        
        position_left = 23
        position_right = -23

        try: 
            while True:

                # get the recognised block IDs
                self.cam.getLatestBlocks()
                centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
                #print("centerLineBlock")

                leftLineBlock = self.cam.isInView(self.leftLineID)
                rightLineBlock = self.cam.isInView(self.rightLineID)

                color_ids = [self.centerLineID, self.leftLineID, self.rightLineID] 
                line_markers = [centerLineBlock, leftLineBlock, rightLineBlock]
                
                # Find which color box is the biggest # of red, blue, purple
                largest_idx = self.largest_block(centerLineBlock, leftLineBlock, rightLineBlock)
                
                # There are no markers if largest_idx = -1, otherwise largest_idx = 0 or 1 or 2
                if largest_idx >= 0: # when largest block is visible                    
                
                    correction = 0
                    
                    # Find the offset for the side markers (needs to be measured)
                    if largest_idx == 0:
                        correction = 0
                        speed_input = speed

                    # Left marker
                    elif largest_idx == 1:
                        correction = 23 #offset
                        speed_input = 1*speed

                    # Right  marker
                    else:
                        correction = -23
                        speed_input = 1*speed
                    
                    # Get the parameters of the largest block
                    self.getBlockParams(line_markers[largest_idx])

                    # Calculate the error corresponding to the offset (offset ~ 23)
                    CL_angular_error = self.blockAngle[-1] + correction # angular correction # constant offset
                    
                    ##########################################################


                    #visAngularError, newServoPosition = self.visTrack(line_markers[largest_idx])
				            #error = -visAngularError-newServoPosition 
				            #bias = self.lanectrl.update(error)



                    # Account for the rotation of the camera ???????????????
                    #camera_rotation = -(servo_pos/50) * 25
                    #angle = CL_angular_error + camera_rotation
                    #lineSteering = angle * 0.015
                    
                    
                        
                    # Follow the red marker with the camera if it is present
                    if largest_idx == 0:
                        ang_err, servo_pos = self.visTrack(centerLineBlock)
                        error = -ang_err-servo_pos-5
                        lineSteering = self.biasControl.update(error)
                        #lineSteering = self.biasControl.update(angle)
                        #print("following red because I can see red")
                    # Otherwise set the camera to center position
                    
                    #elif largest_idx == 1:
                    
                        ##ang_err, servo_pos = self.visTrack(leftLineBlock)
                        #error = CL_angular_error  # + camera_rotation
                        #lineSteering = self.biasControl.update(error)
                        
                        ##lineSteering = angle * 0.015
                        ##servo_pos = self.bot.setServoPosition(0)
                        ##lineSteering = self.biasControl.update(angle)
                        
                    elif largest_idx == 2:
                    
                        ###ang_err, servo_pos = self.visTrack(rightLineBlock)
                        
                        #self.getBlockParams(centerLineBlock)
                        #position_center = self.position[-1]
                        #print("left position")
                        
                        #self.getBlockParams(rightLineBlock)
                        #position_right = self.position[-1]
                        #print("right position")
                        
                        #position = position_right + 20
                        #print("getting position average")
                        #lineSteering = self.biasControl.update(position)
                        #print("using position average for steering")
                        #servo_pos = self.bot.setServoPosition(-10)
                        
                        error = CL_angular_error - 15 #+ camera_rotation #15
                        lineSteering = self.biasControl.update(error)
                        ###servo_pos = self.bot.setServoPosition(10)
                        
                        #lineSteering = angle * 0.015
                        #servo_pos = self.bot.setServoPosition(0)
                        #lineSteering = self.biasControl.update(angle)
                        
                    elif largest_idx == 1:
                    
                        ###ang_err, servo_pos = self.visTrack(rightLineBlock)
                        
                        #self.getBlockParams(centerLineBlock)
                        #position_center = self.position[-1]
                        #print("left position")
                        
                        #self.getBlockParams(rightLineBlock)
                        #position_right = self.position[-1]
                        #print("right position")
                        
                        #position = position_right + 20
                        #print("getting position average")
                        #lineSteering = self.biasControl.update(position)
                        #print("using position average for steering")
                        #servo_pos = self.bot.setServoPosition(-10)
                        
                        error = CL_angular_error + 15 #+ camera_rotation #40
                        lineSteering = self.biasControl.update(error)
                        
                    
                    else:
                        print("red is not the largest block")
                        servo_pos = self.bot.setServoPosition(0)
                        
                        self.getBlockParams(leftLineBlock)
                        position_left = self.position[-1]
                        #print("left position")
                        
                        self.getBlockParams(rightLineBlock)
                        position_right = self.position[-1]
                        #print("right position")
                        
                        position = (position_left - position_right)/2 + 10
                        print("getting position average")
                        lineSteering = self.biasControl.update(position)
                        print("using position average for steering")
                        servo_pos = self.bot.setServoPosition(-10)
                        
                    

                    #reverse = True
                    print('Following', names[largest_idx])

                else:
                    
                    speed_input,lineSteering = 0,0

                
        except KeyboardInterrupt:
            print("ended")
            self.bot.setServoPosition(0)
            self.drive(0,0)
