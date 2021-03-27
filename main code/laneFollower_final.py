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
        self.obstacleID     = 4
        self.obstacleCourseStartID = 5

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
    
    def trackLine(self, blockList):
        count = len(blockList)
        if count < 1:
            self.bot.setServoPosition(0)
            return -1
        else:
            for i in range(count):
                pixelError =  (self.cam.newBlocks[blockList[i]].m_x - self.cam.pixyCenterX) / count
            visAngularError = -(pixelError/self.cam.pixyMaxX*self.cam.pixyX_FoV) # error converted to angle
            visTargetAngle = self.bot.servo.lastPosition + self.bot.gimbal.update(visAngularError) # error relative to pixycam angle
            newServoPosition = self.bot.setServoPosition(visTargetAngle)
            return visAngularError, newServoPosition
            
    def trackBlock(self, blockList):
        count = len(blockList)
        if count < 1:
            self.bot.setServoPosition(0)
            return -1     
        else:
            target = -1
            for i in range(count):
                idx = blockList[i]
                if self.cam.newBlocks[idx].m_y > 0.2 * self.cam.pixyMaxY:
                    target
                    angv, serpos = self.visTrack(idx)
                    return angv, serpos
            
            return 0, 0

        
                
    # output            - none
    # speed             - general speed of bot
    def follow(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop
        
        camSteering = 5.0 # cam rotating angular velocity
        newSerPos = 0 # initialize variable

        focal_length = 300
        
        while True:
            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID) # try find centreline
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)
            centerList = self.cam.getBlockList(self.centerLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID)
            obstacleCourseStartBlock = self.cam.isInView(self.obstacleCourseStartID)

            if obstacleCourseStartBlock >= 0:
                _, servo_pos = self.visTrack(obstacleCourseStartBlock)
                self.getBlockParams(obstacleCourseStartBlock)
                angular_error = self.blockAngle[-1]
                camera_rotation = -(servo_pos/70) * 25
                angle = angular_error + camera_rotation
                lineSteering = angle/(self.cam.pixyY_FoV)
                self.drive(speed, lineSteering)
                continue
            
            if obstacleBlock >= 0:
                obstacle_width = self.cam.newBlocks[obstacleBlock].m_width
                obstacle_distance = (focal_length * 50) / obstacle_width
                print("dist: ", obstacle_distance)
                print(obstacleBlock)
            
            if obstacleBlock >= 0 and abs(obstacle_distance) < 350:
                obsteering = self.avoidObstacle(obstacleBlock, leftLineBlock, rightLineBlock)
                self.drive(speed, obsteering)
                continue
            #else:
            #    self.bot.setServoPosition(0)
            #    self.drive(speed,0) 
            #    continue
            
            if centerLineBlock >= 0: # drive while we see a line
                visAngErr, newSerPos = self.trackBlock(centerList) # Let camera track the centeral block
                steering = -0.6 * newSerPos / 70
                # self.getBlockParams(centerLineBlock)
                # steering = 0.65 * self.blockAngle[-1] / self.cam.pixyX_FoV
                self.drive(speed,steering)
                ###

            
            elif leftLineBlock >= 0 and self.cam.newBlocks[leftLineBlock].m_width >= 2.0 * self.cam.newBlocks[leftLineBlock].m_height and self.cam.newBlocks[leftLineBlock].m_y > 0.7 * 208: # only see the leftlane
                newSerPos -= camSteering
                newSerPos = min(-60, newSerPos)
                self.bot.setServoPosition(newSerPos)
                steering = -0.4* newSerPos / 70
                self.drive(speed, steering)
                
            elif rightLineBlock >= 0 and self.cam.newBlocks[rightLineBlock].m_width >= 2.0 * self.cam.newBlocks[rightLineBlock].m_height and self.cam.newBlocks[rightLineBlock].m_y > 0.7 * 208: # only see the right lane
                newSerPos += camSteering
                newSerPos = max(60, newSerPos)
                self.bot.setServoPosition(newSerPos)
                steering = -0.4 * newSerPos / 70
                self.drive(speed, steering)
                

            else: 
                #newSerPos = 0
                newSerPos *= 0.9
                self.bot.setServoPosition(newSerPos)
                steering = -0.3 * newSerPos / 70
                self.drive(speed, steering)
                
            
            
        return
    
    def avoidObstacle(self, obstacleBlock, LeftBlock, RightBlock):
        _, servo_pos = self.visTrack(obstacleBlock)
        if abs(servo_pos) > 30:
            self.bot.setServoPosition(0)

        bias = 0.08 # bias for steering

        if servo_pos > 0:
            obstacleSteering = -(servo_pos - 70)/70
            print(obstacleSteering)

        elif servo_pos < 0:
            obstacleSteering = -(servo_pos + 70)/70
            print(obstacleSteering)

        obstacleSteering *= bias
                    
        return obstacleSteering
    
        
