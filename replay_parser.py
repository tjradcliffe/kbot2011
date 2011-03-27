from mappings import *

import sys

# convenience object for reporting parse errors
class ParseException(Exception):
    def __init__(self, strMessage, nLine):
        self.strMessage = strMessage
        self.nLine = nLine
        
""" Parser to generate replay file from a specification.

This file defines the little language that specifies robot
behaviour.  The valid commands are as follows (where <SPEED> is
a floating point number between -1 and 1 and <TIME> is a floating
point number between 0.0 and 15.0, and the language is case-insensitive:

1) Driving

DRIVE <DIRECTION> AT <SPEED> FROM <TIME> TO <TIME>

where <DIRECTION> is one of LEFT, RIGHT, FORWARD, BACKWARD

2) Turning

TURN <ORIENTATION> AT <SPEED> FROM <TIME> TO <TIME>

where <ORIENTATION> is one of LEFT or RIGHT

3) Tube control

TUBE <ACTION> AT <SPEED> FROM <TIME> TO <TIME>

where <ACTION> is one of IN, OUT, ROLLUP, ROLLDOWN

4) Arm control

ARM <DIRECTION>  AT <SPEED> FROM <TIME> TO <TIME>

where <DIRECTION> is one of UP or DOWN

5) Wrist control

WRIST <ACTION> AT <TIME>

where <ACTION> is one of IN or OUT

6) Jaw control

JAW OPEN FROM <TIME> TO <TIME>

(Jaw is default closed)

7) Arm presets

ARM <POSITION> AT <TIME>

where <POSITION> is one of PARKED, MIDDLE, LOW, HIGH

(this will have the effect of pressing the appropriate arm preset button
for half a second.)

8) Line following

FOLLOW FROM <TIME> TO <TIME>

"""
#DRIVE <DIRECTION> AT <SPEED> FROM <TIME> TO <TIME>
#where <DIRECTION> is one of LEFT, RIGHT, FORWARD, BACKWARD
def parseDrive(lstLine, lstAxes, lstButtons, nLine):
    strDirection = lstLine[1].upper()
    if "LEFT" == strDirection:
        nAxis = knX
        nSign = -1 # left is negative X
    elif "RIGHT" == strDirection:
        nAxis = knX
        nSign =  1 # right is positive X
    elif "FORWARD" == strDirection or "FORWARDS" == strDirection:
        nAxis = knY
        nSign =  1 # forward is positive Y
    elif "BACKWARD" == strDirection or "BACKWARDS" == strDirection:
        nAxis = knY
        nSign = -1 # backward is negative Y
    else:
        raise ParseException("Did not recognize DIRECTION: "+lstLine[1], nLine)

    try:    # parse out the floating point parameters
        fSpeed = float(lstLine[3])
        fStart = float(lstLine[5])
        fStop = float(lstLine[7])
    except Exception, e:
        raise ParseException("Could not convert one of speed, start or stop time to float", nLIne)
        
    # now fill in the appropriate array
    nStart = int(50*fStart)
    nStop = int(50*fStop)
    for nI in range(nStart, nStop):
        lstAxes[nAxis][nI] = nSign*fSpeed
            
# TURN <ORIENTATION> AT <SPEED> FROM <TIME> TO <TIME>
# where <ORIENTATION> is one of LEFT or RIGHT
def parseTurn(lstLine, lstAxes, lstButtons, nLine):
    strOrientation = lstLine[1].upper()
    if "LEFT" == strOrientation:
        nAxis = knX
        nSign = -1 # left is negative X
    elif "RIGHT" == strOrientation:
        nAxis = knX
        nSign =  1 # right is positive X
    else:
        raise ParseException("Did not recognize ORIENTATION: "+lstLine[1], nLine)

    try:    # parse out the floating point parameters
        fSpeed = float(lstLine[3])
        fStart = float(lstLine[5])
        fStop = float(lstLine[7])
    except Exception, e:
        raise ParseException("Could not convert one of speed, start or stop time to float", nLIne)
        
    # now fill in the appropriate array
    nStart = int(50*fStart)
    nStop = int(50*fStop)
    for nI in range(nStart, nStop):
        lstAxes[nAxis][nI] = nSign*fSpeed
               

# TUBE <ACTION> AT <SPEED> FROM <TIME> TO <TIME>
# where <ACTION> is one of IN, OUT, ROLLUP, ROLLDOWN
def parseTube(lstLine, lstAxes, lstButtons, nLine):
    strAction = lstLine[1].upper()
    if "IN" == strAction:
        nAxis = knRollInOut
        nSign = -1
    elif "OUT" == strAction:
        nAxis = knRollInOut
        nSign = 1
    elif "ROLLUP" == strAction or "UP" == strAction:
        nAxis = knRollAround
        nSign = 1
    elif "ROLLDOWN" == strAction or "DOWN" == strAction:
        nAxis = knRollAround
        nSign = -1
    else:
        raise ParseException("Did not recognize ACTION: "+lstLine[1], nLine)

    try:    # parse out the floating point parameters
        fSpeed = float(lstLine[3])
        fStart = float(lstLine[5])
        fStop = float(lstLine[7])
    except Exception, e:
        raise ParseException("Could not convert one of speed, start or stop time to float", nLIne)
        
    # now fill in the appropriate array
    nStart = int(50*fStart)
    nStop = int(50*fStop)
    for nI in range(nStart, nStop):
        lstAxes[nAxis][nI] = nSign*fSpeed        
    
# ARM <DIRECTION>  AT <SPEED> FROM <TIME> TO <TIME>
# where <DIRECTION> is one of UP or DOWN
# OR
# ARM <POSITION> AT <TIME>
# where <POSITION> is one of PARKED, MIDDLE, LOW, HIGH
def parseArm(lstLine, lstAxes, lstButtons, nLine):
    strCommand = lstLine[1].upper()
    if "UP" == strCommand:
        nAxis = knArmUpDown
        nSign = 1 # up is positive Y
    elif "DOWN" == strCommand:
        nAxis = knArmUpDown
        nSign =  -1 # down is negative Y
    elif "BOTTOM" == strCommand or "LOW" == strCommand:
        nButton = knArmLow
    elif "MIDDLE" == strCommand:
        nButton = knArmMiddle
    elif "TOP" == strCommand or "HIGH" == strCommand:
        nButton = knArmHigh        
    else:
        raise ParseException("Did not recognize DIRECTION or POSITION: "+lstLine[1], nLine)

    try:    # parse out the floating point parameters
        if "UP" == strCommand or "DOWN" == strCommand:
            fSpeed = float(lstLine[3])
            fStart = float(lstLine[5])
            fStop = float(lstLine[7])
            nStart = int(50*fStart)
            nStop = int(50*fStop)
            for nI in range(nStart, nStop):
                lstAxes[nAxis][nI] = nSign*fSpeed           
        else:
            fStart = float(lstLine[3])
            fStop = fStart + 0.5
            nStart = int(50*fStart)
            nStop= int(50*fStop)
            for nI in range(nStart, nStop):
                lstButtons[nButton][nI] = 1                       
    except Exception, e:
        print e
        raise ParseException("Could not convert one of speed, start or stop time to float", nLine)
           
    
# WRIST <ACTION> AT <TIME>
# where <ACTION> is one of IN or OUT
def parseWrist(lstLine, lstAxes, lstButtons, nLine):
    strAction = lstLine[1].upper()
    if "IN" == strAction:
        nButton = knWristIn
    elif "OUT" == strAction:
        nButton = knWristOut
    else:
        raise ParseException("Did not recognize ACTION "+lstLine[1], nLine)        
    try:
        fStart = float(lstLine[3])
        fStop = fStart + 0.5
        nStart = int(50*fStart)
        nStop= int(50*fStop)
        for nI in range(nStart, nStop):
                lstButtons[nButton][nI] = 1                       
    except Exception, e:
        print e
        raise ParseException("Could not convert one of speed, start or stop time to float", nLine)    

# JAW OPEN FROM <TIME> TO <TIME>
# (Jaw is default closed)
def parseJaw(lstLine, lstAxes, lstButtons, nLine):
    nButton = knJawOpen
    try:
        fStart = float(lstLine[3])
        fStop = float(lstLine[5])
        nStart = int(50*fStart)
        nStop= int(50*fStop)
        for nI in range(nStart, nStop):
                lstButtons[nButton][nI] = 1                       
    except Exception, e:
        print e
        raise ParseException("Could not convert one of start or stop time to float", nLine)    
    
    
# FOLLOW FROM <TIME> TO <TIME>
def parseFollow(lstLine, lstAxes, lstButtons, nLine):
    nButton = knLineFollowButton
    try:
        fStart = float(lstLine[2])
        fStop = float(lstLine[4])
        nStart = int(50*fStart)
        nStop= int(50*fStop)
        for nI in range(nStart, nStop):
                lstButtons[nButton][nI] = 1                       
    except Exception, e:
        print e
        raise ParseException("Could not convert one of start or stop time to float", nLine)    
        

# Figure out what command we have and send it to the appropriate parser
def parseLine(strLine, lstAxes, lstButtons, nLine):
    lstLine = strLine.split() # tokenize on whitespace
    
    strCommand = lstLine[0].upper()
    try:
        if "DRIVE" == strCommand:
            parseDrive(lstLine, lstAxes, lstButtons, nLine)
        elif "TURN" == strCommand:
            parseTurn(lstLine, lstAxes, lstButtons, nLine)
        elif "TUBE" == strCommand:
            parseTube(lstLine, lstAxes, lstButtons, nLine)
        elif "ARM" == strCommand:
            parseArm(lstLine, lstAxes, lstButtons, nLine)
        elif "WRIST" == strCommand:
            parseWrist(lstLine, lstAxes, lstButtons, nLine)
        elif "JAW" == strCommand:
            parseJaw(lstLine, lstAxes, lstButtons, nLine)
        elif "FOLLOW" == strCommand:
            parseFollow(lstLine, lstAxes, lstButtons, nLine)
        else:
            print "Could not find valid command at start of line: "+str(nLine)
            print "Should be one of DRIVE/TURN/TUBE/ARM/WRIST/JAW/FOLLOW"
            print strLine
    except ParseException, e:
        print "Encountered a problem with the input file:"
        print ">>> "+e.strMessage
        print "On line: "+str(e.nLine)
        print strLine
        sys.exit(-1)
    except Exception, e:
        print "Encountered a problem with the parser (probably):"
        print ">>> "+str(e)
        print "Triggered by line: "+str(nLine)
        print strLine
        sys.exit(-1)        
