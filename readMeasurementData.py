"""
Author:     Johannes Hauck
Source:
Content:    File includes functions to read data from a measurement .txt file and transform the data.
"""


# find positions of all ch in word
# input:    - word (string): word in which ch should searched
#           - ch (char): character which is to search
# output:   - (array) with positions of char in word
def findAll(word, ch):
    return [i for i, letter in enumerate(word) if letter == ch]


# transform control points from string to tupel
# input:    - cpString (string): control and Bézierpoints sting from measurement file
# output:   - (tupel) with control and Bézierpoints
def getControlBezPoints(cpString):
    comma = findAll(cpString, ',')
    return float(cpString[1:comma[0]]), float(cpString[comma[0] + 2:comma[1]]), float(cpString[comma[1] + 2:len(cpString)-2])


# transform 1x4 vector von from string to tupel
# e.g.: In: '1.0, 2.0, 3.0, 55.0'; Out: (1.0, 2.0, 3.0, 55.0)
# input:    - tupelString (string): string of one row of camera pose
# output:   - (tupel array) of transformed rows camera pose
def get1x4Tupel(tupelString):
    komma = findAll(tupelString, ',')
    return float(tupelString[0:komma[0]]), float(tupelString[komma[0]+2:komma[1]]), float(
        tupelString[komma[1]+2:komma[2]]), float(tupelString[komma[2]+2:])


# transform pose from string to tupel
# input:    - poseString (string): string of camera pose
# output:   - (array) with camera pose
def getPose(poseString):
    auf = findAll(poseString, '(')
    zu = findAll(poseString, ')')
    zu = findAll(poseString, ')')
    z0 = poseString[auf[1]+1:zu[0]]
    z1 = poseString[auf[2]+1:zu[1]]
    z2 = poseString[auf[3]+1:zu[2]]
    z3 = poseString[auf[4]+1:zu[3]]
    return get1x4Tupel(z0), get1x4Tupel(z1), get1x4Tupel(z2), get1x4Tupel(z3)


# extract time stamp
# input:    - poseString; time stamp + u + velocity + pose
# output:   - (float) with timestamp
def getTimestamp(poseString):
    comma = findAll(poseString, ',')
    return float(poseString[:comma[0]])


# extract u value
# input:    - poseString; time stamp + u + velocity + pose
# output:   - (float) with u value
def getUValue(poseString):
    comma = findAll(poseString, ',')
    return float(poseString[comma[0]+2:comma[1]])


# extract velocity
# input:    - poseString; time stamp + u + velocity + pose
# output:   - (float) with velocity value
def getVValue(poseString):
    comma = findAll(poseString, ',')
    return float(poseString[comma[1]+2:comma[2]])
