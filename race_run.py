from pixyBot import pixyBot
from pixyCam import pixyCam
from control import robotRace
from PIDcontroller import PID_controller

# exercises
def main():
    ### IMPORTANT
    servoCorrection = 0 # put the servo correction for your robot here
    ###

    r = pixyBot(servoCorrection, PID_controller(0.16, 0, 0.06))
    p = pixyCam()
    rR = robotRace(r, p)

    rR.race(0.9)
    #rR.test2angles()
main()