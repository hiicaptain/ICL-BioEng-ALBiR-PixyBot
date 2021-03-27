## library for lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2021

from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
import math

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
#   drive                       - differential drive function that takes bias for right left wheel speed
#   getBlockParams              - get and store the size and anglular error of a selected block
#   visTrack                    - move camera to point at selected block
#   follow                      - move bot along a lane, following markers
class laneFollower(object):
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
        self.bot = bot

        self.bot = bot
        self.cam = cam

        self.centerLineID   = 1
        self.leftLineID     = 2
        self.rightLineID    = 3
        self.pinkblockID    = 4
        self.obstacleID     = 5

        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]
        self.blockWidth = [float('nan') for i in range(nObservations)]
        self.blockHeight = [float('nan') for i in range(nObservations)]

    # output            - none
    # drive             - desired general bot speed (-1~1)
    # bias              - ratio of drive speed that is used to turn right (-1~1)
    def drive(self, drive, bias): # Differential drive function
        if bias > 1:
            bias = 1
        if bias < -1:
            bias = -1
        
        correction = [0,0.1]
        
        maxDrive = 1 # set safety limit for the motors

        totalDrive = drive * maxDrive # the throttle of the car
        diffDrive = bias * totalDrive # set how much throttle goes to steering
        straightDrive = totalDrive - abs(diffDrive) # the rest for driving forward (or backward)

        lDrive = (1.0+correction[0])*(straightDrive + diffDrive)
        rDrive = (1.0+correction[1])*(straightDrive - diffDrive)
        
        self.bot.setMotorSpeeds(lDrive, rDrive)

    # output            - -1 if error
    # blockIdx          - index of block in block list that you want to save parameters for
    def getBlockParams(self, blockIdx):
        if (self.cam.newCount-1) < blockIdx or blockIdx < 0: # do nothing when block doesn't exist
            return -1
        else:
            pixelSize = self.cam.newBlocks[blockIdx].m_width;
            pixelSize2 =  self.cam.newBlocks[blockIdx].m_height;
            angleSize = pixelSize/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular size of block
            pixelError = self.cam.newBlocks[blockIdx].m_x -  self.cam.pixyCenterX
            angleError = pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV #get angular error of block relative to front

            # save params
            self.blockSize.append(angleSize)
            self.blockAngle.append(angleError)
            self.frameTimes.append(time())
            self.blockWidth.append(pixelSize)
            self.blockHeight.append(pixelSize2)
        
            # remove oldest params
            self.blockSize.pop(0)
            self.blockAngle.pop(0)
            self.frameTimes.pop(0)
            self.blockWidth.pop(0)
            self.blockHeight.pop(0)
            
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
    
    def visTrackPlus(self, blockIdxList):
        if blockIdxList[0] < 0: # do nothing when block doesn't exist
            self.bot.setServoPosition(0)
            return -1
        else:
            pixelErrorAll = []
            for index in blockIdxList:
                pixelErrorAll.append(self.cam.newBlocks[index].m_x - self.cam.pixyCenterX) # the sum error of all blocks
            pixelError = sum(pixelErrorAll)/len(blockIdxList)
            visAngularError = -(pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV) # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition

    # output            - none
    # speed             - general speed of bot
    def follow(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop
        discovery = 0;
        change = 1;

        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)

            if centerLineBlock >= 0 and self.cam.newBlocks[centerLineBlock].m_y > 0.6 * 208: # drive while we see a line
                discovery = 0;
                print('I can see the red line')
                visAngularError, NextServoRealPosition = self.visTrack(centerLineBlock) 
                bias = -NextServoRealPosition*0.015
                self.drive(speed, bias)
            elif centerLineBlock< 0 and rightLineBlock < 0 and leftLineBlock >= 0 and self.cam.newBlocks[leftLineBlock].m_y > 0.6 * 208:
                discovery = 0;
                print('blue blocks')
                bias = 0.3
                self.drive(speed*0.8, bias)
                NextServoRealPosition = self.bot.setServoPosition(-45)
            elif centerLineBlock < 0 and leftLineBlock < 0 and rightLineBlock >= 0 and self.cam.newBlocks[rightLineBlock].m_y > 0.6 * 208:
                discovery = 0;
                print('purple blocks')  
                bias = -0.3
                self.drive(speed*0.8, bias)
                NextServoRealPosition = self.bot.setServoPosition(45)
            elif centerLineBlock < 0 and leftLineBlock >= 0 and rightLineBlock >= 0:
                discovery = 0
                print('go ahead')
                self.getBlockParams(leftLineBlock)
                self.getBlockParams(rightLineBlock)
                visAngularError = (self.blockAngle[-1]+self.blockAngle[-2])/2
                visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
                NextServoRealPosition = self.bot.setServoPosition(visTargetAngle)
                bias = -NextServoRealPosition*0.1
                self.drive(speed*0.7, bias)
            else:
                discovery = discovery + 1*change
                print('no blocks' + str(discovery))
                servoposition = 0 + 2*discovery
                NextServoRealPosition = self.bot.setServoPosition(servoposition)
                if abs(servoposition) >= 40:
                    change = - change
                self.drive(speed*0.7, 0)
                    
        return

    # output            - none
    # speed             - track the centre of red line
    def follow2(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop

        i = 0;
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.allInView(self.centerLineID) # try find all red block
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)

            if centerLineBlock[0] >= 0: # drive while we see a line
                print('I can see the red line')
                visAngularError, NextServoRealPosition = self.visTrackPlus(centerLineBlock) 
                bias = -NextServoRealPosition*0.02
                self.drive(speed, bias)
            elif centerLineBlock[0] < 0 and rightLineBlock < 0 and leftLineBlock >= 0:
                print('blue blocks')
                bias = 0.35
                self.drive(speed*0.5, bias)
                NextServoRealPosition = self.bot.setServoPosition(-35)
            elif centerLineBlock[0] < 0 and leftLineBlock < 0 and rightLineBlock >= 0:
                print('purple blocks')  
                bias = -0.35
                self.drive(speed*0.5, bias)
                NextServoRealPosition = self.bot.setServoPosition(35)
            elif centerLineBlock[0] < 0 and leftLineBlock >= 0 and rightLineBlock >= 0:
                print('go ahead')
                self.getBlockParams(leftLineBlock)
                self.getBlockParams(rightLineBlock)
                visAngularError = (self.blockAngle[-1]+self.blockAngle[-2])/2
                visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
                NextServoRealPosition = self.bot.setServoPosition(visTargetAngle)
                bias = -NextServoRealPosition*0.008
                self.drive(speed, bias)
            else:
                print('no blocks')
                self.drive(speed, 0)
        return

    # output            - none
    # speed             - filter
    def follow3(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop
        discovery = 0;
        change = 1;

        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)

            if centerLineBlock >= 0 and self.cam.newBlocks[centerLineBlock].m_y > 0.35 * 208: # drive while we see a line
                discovery = 0;
                print('I can see the red line')
                visAngularError, NextServoRealPosition = self.visTrack(centerLineBlock) 
                bias = -NextServoRealPosition*0.015
                self.drive(speed, bias)
            elif centerLineBlock< 0 and rightLineBlock < 0 and leftLineBlock >= 0 and self.cam.newBlocks[leftLineBlock].m_y > 0.35 * 208:
                discovery = 0;
                print('blue blocks')
                ratio = self.cam.newBlocks[leftLineBlock].m_width/self.cam.newBlocks[leftLineBlock].m_height
                visAngularError = -5*ratio
                visTargetAngle = self.bot.servo.lastPosition+visAngularError
                NextServoRealPosition = self.bot.setServoPosition(visTargetAngle)
                bias = 0.04*ratio
                self.drive(speed*0.8, bias)
            elif centerLineBlock < 0 and leftLineBlock < 0 and rightLineBlock >= 0 and self.cam.newBlocks[rightLineBlock].m_y > 0.35 * 208:
                discovery = 0;
                print('purple blocks')  
                ratio = self.cam.newBlocks[rightLineBlock].m_width/self.cam.newBlocks[rightLineBlock].m_height
                visAngularError = 5*ratio
                visTargetAngle = self.bot.servo.lastPosition+visAngularError
                NextServoRealPosition = self.bot.setServoPosition(visTargetAngle)
                bias = -0.04*ratio
                self.drive(speed*0.8, bias)
            elif centerLineBlock < 0 and leftLineBlock >= 0 and rightLineBlock >= 0:
                discovery = 0
                print('go ahead')
#                self.getBlockParams(leftLineBlock)
#                self.getBlockParams(rightLineBlock)
#                visAngularError = (self.blockAngle[-1]+self.blockAngle[-2])/2
#                visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
#                NextServoRealPosition = self.bot.setServoPosition(visTargetAngle)
#                bias = -NextServoRealPosition*0.05
                self.drive(speed, 0)
            else:
                discovery = discovery + 1*change
                print('no blocks' + str(discovery))
                servoposition = 0 + 2*discovery
                NextServoRealPosition = self.bot.setServoPosition(servoposition)
                if abs(servoposition) >= 30:
                    change = - change
                self.drive(speed, 0)
                    
        return
        
        
# output            - none
    # speed             - avoid obstacle
    def remove2(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop
        focal_length = 300
        obstacle_width = 40 

        i = 0;
        while True:
            self.cam.getLatestBlocks()                                                   
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            obstacleBlock = self.cam.allInView(self.obstacleID)
            NextServoRealPosition = 0
            bias = 0;
            
            if len(obstacleBlock) == 1: 
                if obstacleBlock[0] >0:
                    distance =  focal_length* obstacle_width / self.cam.newBlocks[obstacleBlock[0]].m_width 
                    print('I can see the block, distance is ' + str(distance))
                    if abs(NextServoRealPosition) < 70:
                        visAngularError, NextServoRealPosition = self.visTrack(obstacleBlock[0])
                        if distance <= 100:
                            if NextServoRealPosition < 0:
                                bias = (-70 - NextServoRealPosition)*0.01
                            else:
                                bias = (70 - NextServoRealPosition)*0.01
                        else:
                            bias = 0
                        self.drive(speed, bias)
                    else:
                        if NextServoRealPosition < 0:
                            newServoPosition = self.bot.setServoPosition(-50)
                        else:
                            newServoPosition = self.bot.setServoPosition(50)
                        self.drive(speed, 0)
                elif obstacleBlock[0] < 0 and rightLineBlock < 0 and leftLineBlock >= 0:
                    print('blue blocks')
                    invaild = self.getBlockParams(leftLineBlock)
                    if invaild == -1:
                        self.drive(0, 0)
                        print('fail to get left blockparams')
                    else:
                        bias = 0.3
                        self.drive(speed*0.5, bias)
                        newServoPosition = self.bot.setServoPosition(-35)
                elif obstacleBlock[0] < 0 and leftLineBlock < 0 and rightLineBlock >= 0:
                    print('purple blocks')
                    invaild = self.getBlockParams(rightLineBlock)
                    if invaild == -1:
                        self.drive(0, 0)
                        print('fail to get right blockparams')
                    else:
                        bias = -0.3
                        self.drive(speed*0.5, bias)
                        newServoPosition = self.bot.setServoPosition(35)
                elif obstacleBlock[0] < 0 and leftLineBlock >= 0 and rightLineBlock >= 0:
                    self.getBlockParams(leftLineBlock)
                    self.getBlockParams(rightLineBlock)
                    visAngularError = self.blockAngle[-1]+self.blockAngle[-2]
                    visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
                    NextServoPosition = self.bot.setServoPosition(visTargetAngle)
                    bias = -NextServoRealPosition*0.008
                    print('go ahead')
                    self.drive(speed, bias)
                else:
                    self.bot.setServoPosition(0)
                    print('no blocks')
                    self.drive(speed, 0) 
            elif len(obstacleBlock) > 1:
                print('the second obstacle')
                distance =  focal_length* obstacle_width / self.cam.newBlocks[obstacleBlock[1]].m_width 
                print('I can see the block, distance is ' + str(distance))
                visAngularError, NextServoRealPosition = self.visTrack(obstacleBlock[1])
                bias = -NextServoRealPosition*0.02
                self.drive(speed, bias)
        return
        
    # output            - none
    # speed             - distance test
    def Obstacles(self, speed):
    
        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop
        while ture:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID) # try find obstacle
      
            obstacle_width = 40 # mm
            focal_length = 300 # pixel
      
            if obstacleBlock >= 0: 
                distance =  focal_length* obstacle_width / self.cam.newBlocks[obstacleBlock].m_width 
                visAngularError, NextServoRealPosition = self.visTrack(obstacleBlock)
                print(distance)
                       	
        return
        
    # output            - none
    # speed             - avoid obstacle
    def remove(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop
        focal_length = 300
        obstacle_width = 40 
        obstaclemark = 0;

        i = 0;
        while True:
            self.cam.getLatestBlocks()                                                   
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID)
            NextServoRealPosition = 0
            bias = 0;
            if obstaclemark == 0 and obstacleBlock >=0:
                obstaclemark = 1
            elif obstaclemark ==1 and obstacleBlock < 0:
                obstaclemark = 0
            else:
                obstaclemark = obstaclemark
            
            if obstacleBlock >= 0: 
                distance =  focal_length* obstacle_width*0.6 / self.cam.newBlocks[obstacleBlock].m_width 
                print('I can see the block, distance is ' + str(distance))
                if abs(NextServoRealPosition) < 50:
                    visAngularError, NextServoRealPosition = self.visTrack(obstacleBlock)
                    if distance <= 70:
                        print('plan A')
                        if NextServoRealPosition < 0:
                            #bias = (-50 - NextServoRealPosition)*0.01
                            bias = -0.1
                        else:
                            #bias = (50 - NextServoRealPosition)*0.01
                            bias = 0.1
                    else:
                        print('plan B')
                        bias = 0
                    self.drive(speed, bias)
                else:
                    print('plan C')
                    bias = -NextServoRealPosition*0.02
                    self.drive(speed, bias)
            elif obstacleBlock < 0 and rightLineBlock < 0 and leftLineBlock >= 0:
                print("blue") 
                self.getBlockParams(leftLineBlock)
                CL_angular_error = self.blockAngle[-1] + 16
                camera_rotation = -(NextServoRealPosition/70) * 25
                angle = CL_angular_error + camera_rotation
                lineSteering = angle/(self.cam.pixyY_FoV/2.0)
            elif obstacleBlock < 0 and leftLineBlock < 0 and rightLineBlock >= 0:
                print("purple") 
                self.getBlockParams(rightLineBlock)
                CL_angular_error = self.blockAngle[-1] - 16
                camera_rotation = -(NextServoRealPosition/70) * 25
                angle = CL_angular_error + camera_rotation
                lineSteering = angle/(self.cam.pixyY_FoV/2.0)
            elif obstacleBlock < 0 and leftLineBlock >= 0 and rightLineBlock >= 0:
                self.getBlockParams(leftLineBlock)
                self.getBlockParams(rightLineBlock)
                visAngularError = (self.blockAngle[-1]+self.blockAngle[-2])/2
                visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
                NextServoRealPosition = self.bot.setServoPosition(visTargetAngle)
                bias = -NextServoRealPosition*0.01
                print('go ahead')
                self.drive(speed, bias)
            else:
                self.bot.setServoPosition(0)
                print('no blocks')
                self.drive(speed, 0)
                
        return