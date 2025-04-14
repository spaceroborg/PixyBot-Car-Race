## run script for lane following

from pixyBot import pixyBot
from pixyCam import pixyCam
from follower import laneFollower
from PIDcontroller import PID_controller

# exercises
def main():
    ### IMPORTANT
    servoCorrection = 0 # put the servo correction for your robot here
    ###

    r = pixyBot(servoCorrection, PID_controller(0.16, 0, 0.06))
    p = pixyCam()
    lf = laneFollower(r, p)

    lf.follow(0.9)

main()