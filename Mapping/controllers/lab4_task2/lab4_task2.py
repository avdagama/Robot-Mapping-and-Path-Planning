# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from __future__ import print_function
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

#enable distance sensors
frontDistanceSensor = robot.getDistanceSensor('front_ds')
leftDistanceSensor = robot.getDistanceSensor('left_ds')
rightDistanceSensor = robot.getDistanceSensor('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)


# enable camera and recognition
camera = robot.getCamera('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#enable imu
imu = robot.getInertialUnit('inertial unit')
imu.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# get position sensors
leftposition_sensor = robot.getPositionSensor('left wheel sensor')
rightposition_sensor = robot.getPositionSensor('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

def setVelocity(left, right):
    leftMotor.setVelocity(left*(2/1.6))
    rightMotor.setVelocity(right*(2/1.6))
    
def setPosition(left, right):
    leftMotor.setPosition(left*(2/1.6))
    rightMotor.setPosition(right*(2/1.6))
    
def encoderI():
    return[(leftposition_sensor.getValue())*(1.6/2), (rightposition_sensor.getValue())*(1.6/2)]

def getSensors():
    return [leftDistanceSensor.getValue()*39.37, frontDistanceSensor.getValue()*39.37, rightDistanceSensor.getValue()*39.37]

#This grid will be updated as the robot travels the world
# 0:    Empty
# -1:   Wall
# 1-16: Cell number
grid = [[ 0,  0,  0,  0,  0,  0,  0,  0,  0],
        [ 0,  1,  0,  2,  0,  3,  0,  4,  0],
        [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
        [ 0,  5,  0,  6,  0,  7,  0,  8,  0],
        [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
        [ 0,  9,  0, 10,  0, 11,  0, 12,  0],
        [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
        [ 0, 13,  0, 14,  0, 15,  0, 16,  0],
        [ 0,  0,  0,  0,  0,  0,  0,  0,  0]]
        

currCell = 16
lastFrontDistanceSensorValues = getSensors()
waitSteps = 0
startYaw = 0
pi = math.pi
lastPositionValue = 0
distanceToMove = 0
xPos = 7
yPos = 7

currDirection = "North"
robotState = "Stopped in cell"
previousState = ""
visitedCells = []
visitedCells = set(visitedCells)
localizing = True

def printGrid(grid):
    boardColor = '\x1b[5;30;43m'
    wallColor = '\x1b[3;37;47m'
    cellColor = '\x1b[6;30;42m'
    
    resetColor = '\x1b[0m'
    print(boardColor + "\t\t\t\t\t\t\t\t\t" + resetColor)
    for row in grid:
        for index, cell in enumerate(row):
            if (cell == -1):
                if (index%2 == 0):
                    print("   " + wallColor + '|'+ resetColor + "\t", end ='')
                if (index%2 == 1):
                    print( wallColor + "-------" + resetColor + "\t", end ='')
            elif (cell == 0):
                print('' + "\t", end ='')
            elif currCell == cell:
                print("   " + cellColor + str(cell) + resetColor + "\t", end ='')
            elif (cell > 0 and cell < 17):
                print("   " + str(cell) + "\t", end ='')
            else:
                print("err")
            
        print('\n'*4)
    print(boardColor + "\t\t\t\t\t\t\t\t\t" + resetColor)

    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    currYaw = imu.getRollPitchYaw()[2]
    
    if (robotState == "Stopped in cell" and robot.getTime() > 1):
    
        #check to see if it can localize
        recOBJ = camera.getRecognitionObjects()
        if (len(recOBJ) == 1):
            blob = recOBJ[0]
            blobSize = blob.get_size_on_image()
            blobArea = blobSize[0] * blobSize[1]
            blobColor = ""
            if (blob.get_colors()[0] == 1 and blob.get_colors()[1] == 1 and blob.get_colors()[2] == 0):
                blobColor = "Yellow"
            elif (blob.get_colors()[0] == 0 and blob.get_colors()[1] == 1 and blob.get_colors()[2] == 0):
                blobColor = "Green"
            elif (blob.get_colors()[0] == 1 and blob.get_colors()[1] == 0 and blob.get_colors()[2] == 0):
                blobColor = "Red"
            elif (blob.get_colors()[0] == 0 and blob.get_colors()[1] == 0 and blob.get_colors()[2] == 1):
                blobColor = "Blue"
            
            #when a corner cell is found, localize the robot    
            if (blobArea > 700 and localizing):
                localizing = False
                #find current cell
                if (blobColor == "Yellow"):
                    xPos = 1;
                    yPos = 1;
                if (blobColor == "Green"):
                    xPos = 1;
                    yPos = 7;
                if (blobColor == "Red"):
                    xPos = 7;
                    yPos = 1;
                if (blobColor == "Blue"):
                    xPos = 7;
                    yPos = 7;
                #find current direction
                if (currYaw > pi/4 and currYaw < 3*pi/4):
                    currDirection = "North"
                elif (currYaw > 0 and currYaw < pi/4 or currYaw < 0 and currYaw > -pi/4):
                    currDirection = "East"
                elif (currYaw > -3*pi/4 and currYaw < -pi/4):
                    currDirection = "South"
                elif (currYaw > 3*pi/4 or currYaw < -3*pi/4):
                    currDirection = "West"
                
                
            
    
        #forward and back realign/correction
        adjValue = (getSensors()[1] % 10) -2.5
        
        #check for walls
        isFrontWall = getSensors()[1] < 10
        isLeftWall = getSensors()[0] < 10
        isRightWall = getSensors()[2] < 10
        
        if(not localizing):
            #update grid with walls
            if (currDirection == "North"):
                if (isFrontWall):
                    grid[yPos-1][xPos] = -1
                if (isLeftWall):
                    grid[yPos][xPos-1] = -1
                if (isRightWall):
                    grid[yPos][xPos+1] = -1
            elif (currDirection == "East"):
                if (isFrontWall):
                    grid[yPos][xPos+1] = -1
                if (isLeftWall):
                    grid[yPos-1][xPos] = -1
                if (isRightWall):
                    grid[yPos+1][xPos] = -1
            elif (currDirection == "South"):
                if (isFrontWall):
                    grid[yPos+1][xPos] = -1
                if (isLeftWall):
                    grid[yPos][xPos+1] = -1
                if (isRightWall):
                    grid[yPos][xPos-1] = -1
            elif (currDirection == "West"):
                if (isFrontWall):
                    grid[yPos][xPos-1] = -1
                if (isLeftWall):
                    grid[yPos+1][xPos] = -1
                if (isRightWall):
                    grid[yPos-1][xPos] = -1
            
            currCell = grid[yPos][xPos]
            visitedCells.add(currCell)
        
            #get absolute position of walls
            currWalls = []
            #north
            if( grid[yPos-1][xPos] == -1 ):
                currWalls.append("N")
            #east
            if( grid[yPos][xPos+1] == -1 ):
                currWalls.append("E")
            #south
            if ( grid[yPos+1][xPos] == -1 ):
                currWalls.append("S")
            #west
            if ( grid[yPos][xPos-1] == -1) :
                currWalls.append("W")
        
        
            #print data to console
            printGrid(grid)
            print("Current/Last Cell:\t\t" + str(currCell))
            print("Current Robot Pose:\t\t" + "{:.10f}".format(currYaw) + "\t\tDirection:\t" + currDirection)
            
            print("Front Distance Sensor:\t" + str(getSensors()[1]) + "\t\tFront Wall?:\t" + str(isFrontWall))
            print("Left Distance Sensor:\t\t" + str(getSensors()[0]) + "\t\tLeft Wall?:\t" + str(isLeftWall))
            print("Right Distance Sensor:\t" + str(getSensors()[2]) + "\t\tRight Wall?:\t" + str(isRightWall))
            print("Walls Detected (Absolute):\t" + str(currWalls))
            print("Correction Applied:\t\t" + str(adjValue))
        
        
        # MAKE NEXT MOVE
        #move straight
        if (not isFrontWall and robot.getTime() > 2):
            setVelocity(3, 3)
            distanceToMove = 10 + adjValue
            setPosition(encoderI()[0] + distanceToMove, encoderI()[1] + distanceToMove)
            stopPosition = encoderI()[0] + distanceToMove
            robotState = "Moving straight"
            lastMoveWasStraight = True
            #update the current cell
            if (currDirection == "North"):
                yPos -= 2
            elif (currDirection == "East"):
                xPos += 2
            elif (currDirection == "South"):
                yPos += 2
            elif (currDirection == "West"):
                xPos -= 2
        
        #if you can go left and right, choose by random
        elif ((not isLeftWall) and (not isRightWall)):
            #rotate right
            if (robot.getTime()%2 < 1):
                leftMotor.setPosition(float('inf'))
                rightMotor.setPosition(float('inf'))
                setVelocity(3, -3)
                startYaw = imu.getRollPitchYaw()[2]
                robotState = "Rotating right"
            #rotate left    
            else:
                leftMotor.setPosition(float('inf'))
                rightMotor.setPosition(float('inf'))
                setVelocity(-3, 3)
                startYaw = imu.getRollPitchYaw()[2]
                robotState = "Rotating left"
                
        #rotate left
        elif (not isLeftWall):
            leftMotor.setPosition(float('inf'))
            rightMotor.setPosition(float('inf'))
            setVelocity(-3, 3)
            startYaw = imu.getRollPitchYaw()[2]
            robotState = "Rotating left"
            
        #rotate right
        elif (not isRightWall):
            leftMotor.setPosition(float('inf'))
            rightMotor.setPosition(float('inf'))
            setVelocity(3, -3)
            startYaw = imu.getRollPitchYaw()[2]
            robotState = "Rotating right"
            
        #rotate 180 degrees
        else:
            leftMotor.setPosition(float('inf'))
            rightMotor.setPosition(float('inf'))
            setVelocity(-3, 3)
            startYaw = imu.getRollPitchYaw()[2]
            robotState = "Rotating 180"
        
    if (robotState == "Moving straight"):
        
        if (encoderI()[0] >= stopPosition-1):
            robotState = "Stopped in cell"
        
    if (robotState == "Rotating left"):
        
        if (currDirection == "North"):
            if (currYaw < 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "West"
        elif (currDirection == "West"):
            if (currYaw > -pi/2 and currYaw < 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "South"
        elif (currDirection == "South"):
            if (currYaw >= 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "East"
        elif (currDirection == "East"):
            if (currYaw >= pi/2):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "North"
                
    if (robotState == "Rotating right"):
        
        if (currDirection == "South"):
            if (currYaw >= 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "West"
        elif (currDirection == "East"):
            if (currYaw <= -pi/2):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "South"
        elif (currDirection == "North"):
            if (currYaw <= 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "East"
        elif (currDirection == "West"):
            if (currYaw <= pi/2 and currYaw > 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "North"
                
    if (robotState == "Rotating 180"):
        
        if (currDirection == "East"):
            if (currYaw > math.pi - 0.2 and currYaw < -math.pi + 0.2):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "West"
        elif (currDirection == "North"):
            if (currYaw >= (-math.pi/2) and currYaw < (-math.pi/2)+0.2):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "South"
        elif (currDirection == "West"):
            if (currYaw >= 0 and currYaw < 0.5):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "East"
        elif (currDirection == "South"):
            if (currYaw >= (math.pi/2) and currYaw < (math.pi/2)+0.2):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "North"
                
    
    # stop program when all cells are visited
    if (len(visitedCells) == 16):
        setVelocity(0, 0)
        print("\nSuccessfully completed the map in " + str(robot.getTime()) + " seconds!")
        break;
    
    
    if (robotState != previousState):
        if (localizing):
            print("Localizing..."),   
        print("Robot State:\t\t\t" + robotState)
    previousState = robotState
    
    print("\n"*5)
    
    pass

# Enter here exit cleanup code.
