## run script for obstacle avoidance exercises
## made by HTL, LH, DK
## last edited Daniel Ko 22/02/2021

from pixyBot import pixyBot
from pixyCam import pixyCam
from obstacleAvoidance_new import obstacleAvoidance
from PIDcontroller import PID_controller

# exercises
def main():
    ### IMPORTANT
    servoCorrection = -11 # put the servo correction for your robot here
    ###

    r = pixyBot(servoCorrection, PID_controller(0.1, 0, 0.06))
    p = pixyCam()
    oa = obstacleAvoidance(r, p)
    
    #oa.bot.setServoPosition(-45)
    #oa.stopAtStationaryObstacles(0.3)

    oa.avoidStationaryObstacles(0.4)

main()
###
