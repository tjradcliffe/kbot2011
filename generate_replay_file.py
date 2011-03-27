# generate an autonomous replay file from specification

from mappings import *
from replay_parser import parseLine

import sys

sys.argv.append("test_spec.txt")
sys.argv.append("score_one.dat")

# make space for lists and fill them with zeros
nTimePoints = 750 # 750 ticks per auto run (15 seconds)
lstAxes = []
for nI in range(0,10):  # five axes per two controllers
    lstAxis = [0 for nI in range(0,nTimePoints)]
    lstAxes.append(lstAxis)
lstButtons = []
for nI in range(0,20):
    lstButton = [0 for nI in range(0,nTimePoints)]
    lstButtons.append(lstButton) # ten buttons per two controllers

# now read the specification file
inFile = open(sys.argv[1],"r")
nLine = 0
for strLine in inFile:
    nLine += 1  # number lines from 1, counting comments and blanks
    
    strLine = strLine.strip()   # remove leading/trailing whitespace
    if 0 == len(strLine) or '#' == strLine[0]:
        continue    # skip blank lines and coments
        
    parseLine(strLine, lstAxes, lstButtons, nLine) # set state based on commands

inFile.close()

# output the replay file
outFile = open(sys.argv[2], "w")
for nI in range(0, nTimePoints):
    outFile.write(str(nI)+" ")
    for nJ in range(0, len(lstAxes)):
        outFile.write(str(lstAxes[nJ][nI])+" ")
    for nJ in range(0, len(lstButtons)):
        outFile.write(str(lstButtons[nJ][nI])+" ")
    outFile.write("\n")
