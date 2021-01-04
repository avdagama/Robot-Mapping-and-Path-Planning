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


#************************************************************
# SET INITIAL VALUES HERE

#Mark the values in the grid below
# -1    Start Cell
# 0:    Empty
# 1:    Wall/Obstacle
# 2:    Goal

grid = [[ 1,  1,  1,  1,  1,  1,  1,  1,  1],
        [ 1,  2,  0,  0,  0,  0,  0,  0,  1],
        [ 1,  1,  1,  1,  1,  0,  1,  0,  1],
        [ 1,  0,  0,  0,  1,  0,  1,  0,  1],
        [ 1,  0,  1,  0,  1,  0,  1,  0,  1],
        [ 1,  0,  1,  0,  0,  0,  1,  0,  1],
        [ 1,  0,  1,  1,  1,  1,  1,  0,  1],
        [ 1,  0,  0,  0,  0,  0,  0, -1,  1],
        [ 1,  1,  1,  1,  1,  1,  1,  1,  1]]
        
currDirection = "North"
        
#************************************************************
     
currCell = 16
lastFrontDistanceSensorValues = getSensors()
waitSteps = 0
startYaw = 0
pi = math.pi
lastPositionValue = 0
distanceToMove = 0
xPos = 7
yPos = 7
distanceToGoal = 2
startX = 0
startY = 0
goalX = 0
goalY = 0

pathGrid = [[ 0,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0,  0,  0,  0]]


robotState = "Making map"
previousState = ""
visitedCells = []
visitedCells = set(visitedCells)

def printGrid(grid):

    boardColor = '\x1b[5;30;43m'
    wallColor = '\x1b[3;37;47m'
    pathColor = '\x1b[5;30;42m'
    startColor = '\x1b[5;30;44m'
    goalColor = '\x1b[5;30;41m'
    currColor = '\x1b[5;30;45m'
    resetColor = '\x1b[0m'
    
    print(boardColor + "\t\t\t\t\t\t\t\t\t" + resetColor)
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            #walls
            if (cell == 1):
                if (x%2 == 0):
                    print(wallColor + "   " + "|" + '\t' + resetColor, end = "")
                elif (x%2 == 1):
                    print(wallColor + "-------" + resetColor, end = "\t")
            #current cell
            elif (y == yPos and x == xPos):
                print(currColor + "   " + str(cell) + '\t' + resetColor, end = "")
            #start cell
            elif (y == startY and x == startX):
                print(startColor + "   " + str(cell) + '\t' + resetColor, end = "")
            #goal cell
            elif (y == goalY and x == goalX):
                print(goalColor + "   " + str(cell) + '\t' + resetColor, end = "")
            #path
            elif (pathGrid[y][x] == 1):
                print(pathColor + "   " + str(cell) + '\t' + resetColor, end = "")
            else:
                print("   " + str(cell), end = "\t")
            
        print('\n'*4)
    print(boardColor + "\t\t\t\t\t\t\t\t\t" + resetColor)


    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    currYaw = imu.getRollPitchYaw()[2]
    
    # make the map using the WaveFrontPlanner algorithm
    if (robotState == "Making map"):
        for y, row in enumerate(grid):
            for x, cell in enumerate(row):
                #get start cell coords
                if (cell == -1):
                        startX = x
                        startY = y
                        xPos = startX
                        yPos = startY
                #traverse cells and mark distances
                if (cell == distanceToGoal):
                    
                    #north
                    if (grid[y-1][x] == 0 or grid[y-1][x] == -1):
                        grid[y-1][x] = cell + 1
                    #east
                    if (grid[y][x+1] == 0 or grid[y][x+1] == -1):
                        grid[y][x+1] = cell + 1
                    #south
                    if (grid[y+1][x] == 0 or grid[y+1][x] == -1):
                        grid[y+1][x] = cell + 1
                    #west
                    if (grid[y][x-1] == 0 or grid[y][x-1] == -1):
                        grid[y][x-1] = cell + 1
            
        printGrid(grid)
        distanceToGoal += 1
        
        if (robot.getTime() > 2):
            robotState = "Path planning"
    
    #find the shortest path based on the generated map    
    if (robotState == "Path planning"):
        
        northDist = grid[yPos-1][xPos]
        eastDist = grid[yPos][xPos+1]
        southDist = grid[yPos+1][xPos]
        westDist = grid[yPos][xPos-1]
        
        if (northDist == 1):
            northDist = 999
        if (eastDist == 1):
            eastDist = 999
        if (southDist == 1):
            southDist = 999
        if (westDist == 1):
            westDist = 999
        
        # going north is the shortest path
        if (northDist <= eastDist and northDist <= southDist and northDist <= westDist):
            yPos -= 1
        # going east is the shortest path
        elif (eastDist <= northDist and eastDist <= southDist and eastDist <= westDist):
            xPos += 1
        # going south is the shortest path
        elif (southDist <= eastDist and southDist <= northDist and southDist <= westDist):
            yPos += 1
        # going west is the shortest path
        elif (westDist <= eastDist and westDist <= southDist and westDist <= northDist):
            xPos -= 1
        
        #if entire path is plotted, change state
        if (grid[yPos][xPos] == 2):
            goalX = xPos
            goalY = yPos
            xPos = startX
            yPos = startY
            robotState = "Stopped in cell"
            print("START")
        
        pathGrid[yPos][xPos] = 1
        
    
    if (robotState == "Stopped in cell" and robot.getTime() > 1):
        
        #print data to console
        printGrid(grid)
        
        percentComplete = 100 - ((100 * (grid[yPos][xPos] - 2)) / (grid[startY][startX] - 2))
        print(str(percentComplete) + "% Complete")
        
        estimatedTime = (robot.getTime() * ((100 * (grid[yPos][xPos] - 2)) / (grid[startY][startX] - 2)))/100
        print("ETA: " + str(estimatedTime) + " seconds")
        
        #stop program when robot reaches goal cell
        if (yPos == goalY and xPos == goalX):
            setVelocity(0, 0)
            print("GOAL!")
            break;
    
        #forward and back realign/correction
        adjValue = (getSensors()[1] % 5) -2.5
        
        
        
        canMoveNorth = pathGrid[yPos-1][xPos] == 1 or grid[yPos-1][xPos] == 2
        canMoveEast = pathGrid[yPos][xPos+1] == 1 or grid[yPos][xPos+1] == 2
        canMoveSouth = pathGrid[yPos+1][xPos] == 1 or grid[yPos+1][xPos] == 2
        canMoveWest = pathGrid[yPos][xPos-1] == 1 or grid[yPos][xPos-1] == 2
        
        
        canMoveForward = False
        canMoveLeft = False
        canMoveRight = False
        
        if (currDirection == "North"):
            canMoveForward = canMoveNorth
            canMoveLeft = canMoveWest
            canMoveRight = canMoveEast
        elif (currDirection == "East"):
            canMoveForward = canMoveEast
            canMoveLeft = canMoveNorth
            canMoveRight = canMoveSouth
        elif (currDirection == "South"):
            canMoveForward = canMoveSouth
            canMoveLeft = canMoveEast
            canMoveRight = canMoveWest
        elif (currDirection == "West"):
            canMoveForward = canMoveWest
            canMoveLeft = canMoveSouth
            canMoveRight = canMoveNorth
            
            
        # MAKE NEXT MOVE
        #move straight
        if (canMoveForward):
            setVelocity(3, 3)
            distanceToMove = 5 + adjValue
            setPosition(encoderI()[0] + distanceToMove, encoderI()[1] + distanceToMove)
            stopPosition = encoderI()[0] + distanceToMove
            robotState = "Moving straight"
            lastMoveWasStraight = True
            #update the current cell
            if (currDirection == "North"):
                yPos -= 1
            elif (currDirection == "East"):
                xPos += 1
            elif (currDirection == "South"):
                yPos += 1
            elif (currDirection == "West"):
                xPos -= 1
                
        #rotate left
        elif (canMoveLeft):
            leftMotor.setPosition(float('inf'))
            rightMotor.setPosition(float('inf'))
            setVelocity(-3, 3)
            startYaw = imu.getRollPitchYaw()[2]
            robotState = "Rotating left"
            
        #rotate right
        elif (canMoveRight):
            leftMotor.setPosition(float('inf'))
            rightMotor.setPosition(float('inf'))
            setVelocity(3, -3)
            startYaw = imu.getRollPitchYaw()[2]
            robotState = "Rotating right"
            
        
    if (robotState == "Moving straight"):
        
        if (encoderI()[0] >= stopPosition-1):
            robotState = "Stopped in cell"
        
    if (robotState == "Rotating left"):
        
        if (currDirection == "North"):
            if (currYaw >= -(math.pi/2) and currYaw <= -(math.pi/2)+0.5):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "West"
        elif (currDirection == "West"):
            if (currYaw > -0.0 and currYaw < 0.2):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "South"
        elif (currDirection == "South"):
            if (currYaw > (math.pi/2) and currYaw < (math.pi/2)+0.5):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "East"
        elif (currDirection == "East"):
            if (currYaw <= 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "North"
                
    if (robotState == "Rotating right"):
        
        if (currDirection == "South"):
            if (currYaw <= -(math.pi/2)):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "West"
        elif (currDirection == "East"):
            if (currYaw <= 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "South"
        elif (currDirection == "North"):
            if (currYaw <= math.pi/2 and currYaw > math.pi/2 -0.5):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "East"
        elif (currDirection == "West"):
            if (currYaw >= 0):
                setVelocity(0, 0)
                robotState = "Stopped in cell"
                currDirection = "North"
                
    
    # stop program when all cells are visited
    if (len(visitedCells) == 16):
        setVelocity(0, 0)
        print("\nSuccessfully completed the map in " + str(robot.getTime()) + " seconds!")
        break;
        
    
    
    print("\n"*5)
    
    pass

# Enter here exit cleanup code.
