# constants that define the robot.  If sufficiently clever we could
# parse these out of mappings.h
knX = 0                     # axes
knY = 1
knR = 2
knRollAround = 7
knRollInOut = 8
knArmUpDown = 6

knLineFollowButton= 4   # button on driver (xbox) controller

knButtons = 10           # buttons per controller... all these buttons are on 2nd controller
knArmParked = knButtons  # wrist in, arm down, all motors stopped
knArmLow = knButtons+1
knArmMiddle = knButtons+2
knArmHigh = knButtons+3
knWristIn = knButtons+4
knWristOut = knButtons+6
knJawOpen = knButtons+7
