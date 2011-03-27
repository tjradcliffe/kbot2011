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
        nSign =  1 # right is negative X
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
    print "parseTurn"

# TUBE <ACTION> AT <SPEED> FROM <TIME> TO <TIME>
# where <ACTION> is one of IN, OUT, ROLLUP, ROLLDOWN
def parseTube(lstLine, lstAxes, lstButtons, nLine):
    print "parseTube"
    
# ARM <DIRECTION>  AT <SPEED> FROM <TIME> TO <TIME>
# where <DIRECTION> is one of UP or DOWN
# OR
# ARM <POSITION> AT <TIME>
# where <POSITION> is one of PARKED, MIDDLE, LOW, HIGH
def parseArm(lstLine, lstAxes, lstButtons, nLine):
    print "parseArm"
    
# WRIST <ACTION> AT <TIME>
# where <ACTION> is one of IN or OUT
def parseWrist(lstLine, lstAxes, lstButtons, nLine):
    print "parseWrist"

# JAW OPEN FROM <TIME> TO <TIME>
# (Jaw is default closed)
def parseJaw(lstLine, lstAxes, lstButtons, nLine):
    print "parseJaw"
    
# FOLLOW FROM <TIME> TO <TIME>
def parseFollow(lstLine, lstAxes, lstButtons, nLine):
    print "parseFollow"

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
        print "Triggered by line: "+str(e.nLine)
        print strLine
        sys.exit(-1)        
