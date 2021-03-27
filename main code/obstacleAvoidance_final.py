## library for obstacle avoidance and lane following activities
## feel free to add/modify functions if you are comfortable
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2020

from pixyBot import pixyBot
from pixyCam import pixyCam
from time import time
import math

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
    def __init__(self, bot=pixyBot(0), cam=pixyCam()):
        self.bot = bot

        self.bot = bot
        self.cam = cam

        self.centerLineID   = 1
        self.leftLineID     = 2
        self.rightLineID    = 3
        self.obstacleID     = 5

        # tracking parameters variables
        nObservations = 20
        self.frameTimes = [float('nan') for i in range(nObservations)]
        self.blockSize = [float('nan') for i in range(nObservations)]
        self.blockAngle = [float('nan') for i in range(nObservations)]
        self.pixelSize = [float('nan') for i in range(nObservations)]

        self.count = 1 # Keep count of how often the camera rotation is updated

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
            self.pixelSize.append(pixelSize)

            # remove oldest params
            self.blockSize.pop(0)
            self.blockAngle.pop(0)
            self.frameTimes.pop(0)
            self.pixelSize.pop(0)

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
    def stopAtStationaryObstacles(self, speed):

        self.bot.setServoPosition(0) # set servo to centre
        self.drive(0, 0) # set racer to stop
        servo_pos = 0

        while True:
            ###Level 1### 
            # Initial values
            targetSpeed = speed
            steering = 0
            obstacle_width = 40 #mm
            frontal_angular_space = 40

            self.cam.getLatestBlocks()
            centerLineBlock = self.cam.isInView(self.centerLineID)
            obstacleBlock = self.cam.isInView(self.obstacleID) # try find obstacle
            self.getBlockParams(obstacleBlock) # try find obstacle
            
            if obstacleBlock >= 0: # When robot sees obstacle
                _, servo_pos = self.visTrack(obstacleBlock) # Start tracking obstacle
                #object_width = self.cam.newBlocks[obstacleBlock].m_width # in pixels
                average = sum(self.pixelSize)/len(self.pixelSize)
                distance = (self.cam.focal_length * obstacle_width) / average 
                print(distance)

                ###Level 2&3###
                # Distance is in mm
                if distance < 150:
                    # While the servo position is lower than the frontal angular space
                    while abs(servo_pos) < abs(frontal_angular_space):
                        targetSpeed = 0.2 # The speed I want when turning
                        print(servo_pos) # Printing servo position for debugging

                        # Updated blocks seen by camera and track obstacle
                        self.cam.getLatestBlocks()
                        self.getBlockParams(obstacleBlock) # try find obstacle
                        _, servo_pos = self.visTrack(obstacleBlock) # Start tracking obstacle

                        if frontal_angular_space < 0:
                            steering = -0.3
                        else:
                            steering = 0.3

                        self.drive(targetSpeed, steering)

                    # Once we have desired angle, set speed to 0 and exit program
                    targetSpeed = 0
                    return 0

            self.drive(targetSpeed, steering)
        return

    # output            - none
    # speed             - general speed of bot
    def avoidStationaryObstacles(self, speed):

        self.bot.setServoPosition(0)
        obstacle_width = 40 #mm
        targetSpeed = speed
        servo_pos = 0
        old_servo_pos = servo_pos
        distance = 0
        old_distance = distance

        lineSteering = 0
        obstacleSteering = 0

        while True:
            ###Level 4### 
            # Obstacle avoidance
            self.cam.getLatestBlocks()
            self.getBlockParams(self.cam.isInView(self.obstacleID))
            obstacleBlock = self.cam.isInView(self.obstacleID)

            if obstacleBlock >= 0: # When robot sees obstacle
                # Calculating distance to object
                object_width = self.cam.newBlocks[obstacleBlock].m_width # in pixels
                distance = (self.cam.focal_length * obstacle_width) / object_width 

                self.count += 1 # Incrementing counter
                if self.count > 1:
                    _, servo_pos = self.visTrack(obstacleBlock) # Visually track obstacle
                    self.count = 0

                if servo_pos > 0: # Obstacle is to the left
                    obstacleSteering = -(servo_pos - 70)/70
                    #print(obstacleSteering)

                elif servo_pos < 0: # Obstacle is to the right
                    obstacleSteering = -(servo_pos + 70)/70
                    #print(obstacleSteering)

                if abs(distance) > 50:
                    obstacleSteering = 0

            # Lane following
            #cLB1, cLB2 = self.cam.isInView_2(self.centerLineID)
            centerLineBlock = self.cam.isInView(self.centerLineID)
            leftLineBlock = self.cam.isInView(self.leftLineID)
            rightLineBlock = self.cam.isInView(self.rightLineID)

            if centerLineBlock >= 0:
            #if centerLineBlock >= 0: # If center line is in view
                #self.cam.getLatestBlocks()
                #cLB1, cLB2 = self.cam.isInView_2(self.centerLineID)

                _, servo_pos = self.visTrack(cLB2)

                self.getBlockParams(centerLineBlock) # Obtaining params of centre line
                CL_angular_error = self.blockAngle[-1] # Centre line angular error
                camera_rotation = -(servo_pos/70) * 25
                angle = CL_angular_error + camera_rotation
                lineSteering = (angle/(self.cam.pixyY_FoV/2.0))
   
            elif leftLineBlock >= 0:
                print("blue")
                self.getBlockParams(leftLineBlock)
                CL_angular_error = self.blockAngle[-1] + 16
                camera_rotation = -(servo_pos/70) * 25
                angle = CL_angular_error + camera_rotation
                lineSteering = angle/(self.cam.pixyY_FoV/2.0)

            elif rightLineBlock >= 0:
                print("purple") 
                self.getBlockParams(rightLineBlock)
                CL_angular_error = self.blockAngle[-1] - 16
                camera_rotation = -(servo_pos/70) * 25
                angle = CL_angular_error + camera_rotation
                lineSteering = angle/(self.cam.pixyY_FoV/2.0)

            if abs(servo_pos) > 45:
                    self.bot.setServoPosition(0)

            # Saving current servo position and distance
            old_servo_pos = servo_pos
            old_distance = distance

            # Calculating steering bias
            bias = self.calculateSteeringBias(distance)
            #print(distance)

            obstacleSteering = obstacleSteering * bias
            lineSteering = lineSteering * (1 - bias)

            steering = obstacleSteering + lineSteering # we set the final steering as a linear combination of the obstacle steering and center line steering - but it's all up to you!
            #print(steering)
            self.drive(targetSpeed, steering)
            ###

        return

    def calculateSteeringBias(self, distance):
        distance_threshold = 200
        maximum_bias = 0.5
        bias = 0

        if distance < distance_threshold:
            bias = (abs(distance - distance_threshold)/(distance_threshold)) * maximum_bias
        return bias
