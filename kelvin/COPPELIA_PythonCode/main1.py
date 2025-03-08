#!/usr/bin/python
import vision as vs
import time
import cv2
# import the  bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *
import navigation as nav
from state_machine import MAIN_STATE, SUB_STATE

sceneParameters = SceneParameters()

# Starting contents of the bays [shelf,X,Y]. Set to -1 to leave empty.
sceneParameters.bayContents = np.random.random_integers(0,5,sceneParameters.bayContents.shape)
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl
sceneParameters.bayContents[1,1,2] = warehouseObjects.mug
sceneParameters.bayContents[2,3,1] = warehouseObjects.bottle
sceneParameters.bayContents[3,1,2] = warehouseObjects.soccer
sceneParameters.bayContents[4,2,0] = warehouseObjects.rubiks
sceneParameters.bayContents[5,0,1] = warehouseObjects.cereal


#sceneParameters.obstacle0_StartingPosition = [0.5,-0.7]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
# sceneParameters.obstacle0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = [0.5,-0.5]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene


# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'	# specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1		# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the collector in metres
robotParameters.cameraHeightFromFloor = 0.15 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.0 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxItemDetectionDistance = 1 # the maximum distance away that you can detect the items in metres
robotParameters.maxPackingBayDetectionDistance = 2.5 # the maximum distance away that you can detect the packing bay in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres
robotParameters.maxRowMarkerDetectionDistance = 2.5 # the maximum distance away that you can detect the row markers in metres

# Collector Parameters
robotParameters.collectorQuality = 1 # specifies how good your item collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
robotParameters.maxCollectDistance = 0.15 #specificies the operating distance of the automatic collector function. Item needs to be less than this distance to the collector

robotParameters.sync = False # This parameter forces the simulation into sync

warehouseBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)


paused_update = False
def SetSpeed(forward_vel, rotational_vel):
    if not paused_update:
       # warehouseBotSim.SetTargetVelocities(action['forward_vel'],action['rotational_vel'])
        warehouseBotSim.SetTargetVelocities(forward_vel,rotational_vel)
    else:
        pass # do nothing if paused

def Turn45deg(Side):
    paused_update = True # pause the update loop
    if Side == 'left':
        print('Turning Left')
        warehouseBotSim.SetTargetVelocities(0 , -0.2)
        warehouseBotSim.SetTargetVelocities(0 , 0.2)  
        time.sleep(15)
        warehouseBotSim.SetTargetVelocities(0 , 0)
        time.sleep(1)
        paused_update = False # resume the update loop
    elif Side == 'right':
        print('Turning Right')
        warehouseBotSim.SetTargetVelocities(0 , 0.2)
        warehouseBotSim.SetTargetVelocities(0 , -0.2)
        time.sleep(15)
        warehouseBotSim.SetTargetVelocities(0 , 0)
        time.sleep(1)
        paused_update = False # resume the update loop

def MoveForward(delay):
    paused_update = True # pause the update loop
    print('Moving Forward')
    warehouseBotSim.SetTargetVelocities(0.1, 0)
    time.sleep(delay)
    paused_update = False # resume the update loop

# MAIN SCRIPT
robot_state = 'INIT'
action = {}
goal_position = {}
avoid_position = {}
found_row = False
target_aisle = 3
main_state = MAIN_STATE.INIT
sub_state = SUB_STATE.FIND_AISLE

if __name__ == '__main__':
# Wrap everything in a try except case that catches KeyboardInterrupts. 
# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C
    try:
        warehouseBotSim.StartSimulator()
        vision = vs.VisionModule()
        navigation = nav.NavigationModule()
        goal_bay_position = [90, 70, 40, 20] # bay positions in the row in cm
        while True: 
            imgRGB, imgHSV, RobotView = warehouseBotSim.Capturing()
            WallRGB,  WallImgGray, WallMask = vision.findWall(imgHSV,imgRGB)
            ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
            avg_center, avg_bearing, avg_distance, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)
            
            contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            ShelfCenters = vision.GetContoursShelf(contoursShelf, RobotView, (0, 0, 255), "S", Draw=True)
            ShelfCenter, ShelfBearing = vision.GetInfoShelf(RobotView, ShelfCenters, imgRGB)
            corners = vision.ProcessShelfCorners(contoursShelf, RobotView, draw=True)
            print(len(corners))
            contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
            detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)
            ObsTest, ObsBearing = vision.GetInfoObject(RobotView, detected_obstacles, imgRGB)
            if ObsTest is not None:
                print(ObsTest)


            itemsRB, packingBayRB, obstaclesRB, rowMarkerRangeBearing, shelfRangeBearing = warehouseBotSim.GetDetectedObjects(
				[
					warehouseObjects.items,
     				warehouseObjects.shelves,
					warehouseObjects.row_markers,
					warehouseObjects.obstacles,
					warehouseObjects.packingBay,
				]
			)

            if robot_state == 'INIT':
                robot_state = 'SEARCH_FOR_ROW'
                action['forward_vel'] = 0
                action['rotational_vel'] = 0

            elif robot_state == 'SEARCH_FOR_ROW': # Rotate until row marker is found
                if shape_count  == 0:
                    action['rotational_vel'] = -0.2
                    action['forward_vel'] = 0
                elif shape_count  != 0:
                    if abs(avg_bearing) > 20:
                        action['rotational_vel'] = -0.2
                        action['forward_vel'] = 0
                    elif abs(avg_bearing) < 20: # Rotate slower to check the markers carefully
                        action['rotational_vel'] = -0.2
                        action['forward_vel'] = 0
                        if abs(avg_bearing) < 10:
                            action['rotational_vel'] = 0
                            action['forward_vel'] = 0
                            if shape_count == target_aisle:
                                found_row = True
                                print('Row marker found')
                            else :
                                #robot_state = 'LOST'
                                robot_state = 'STOPPING'

                if rowMarkerRangeBearing[2] != None:
                    found_row = True

                if found_row:
                    print('Row marker found')
                    robot_state = 'MOVE_DOWN_ROW'
                    action['rotational_vel'] = 0
                    action['forward_vel'] = 0

            elif robot_state == 'MOVE_DOWN_ROW':
                found_row = False
                print('Moving down the row')
                if rowMarkerRangeBearing[2] != None:
                    found_row = True
                    goal_position['range'] = avg_distance /100
                    goal_position['bearing'] = avg_bearing /100
                if found_row:
                    action = navigation.calculate_goal_velocities(goal_position, ObsTest)
                    print(obstaclesRB)
                if avg_distance < goal_bay_position[3]:
                    robot_state = 'FIND_ITEM'
                    
                    action['forward_vel'] = 0
                    action['rotational_vel'] = 0

                if found_row == False:
                    robot_state = 'SEARCH_FOR_ROW'
                    action['forward_vel'] = 0
                    action['rotational_vel'] = 0

            elif robot_state == 'FIND_ITEM':
                contoursItem, ItemMask = vision.findItems(imgHSV)
                detected_items = vision.GetContoursObject(contoursItem, RobotView, (255, 255, 255), "Obs", Draw=True)
                Turn45deg('left')  # Rotate the robot to search for items

                action['rotational_vel'] = 0 
                action['forward_vel'] = 0
                robot_state = 'PICK_ITEM'
            elif robot_state == 'PICK_ITEM':
                print('Picking up the item')
                action['rotational_vel'] = 0
                action['forward_vel'] = 0.1
                time.sleep(3)
                robot_state = 'FIND_EXIT'
                action['forward_vel'] = 0
            elif robot_state == 'FIND_EXIT':
                if len(ShelfCenter) < 2:
                    print('Looking for exit')
                    action['rotational_vel'] = 0.2
                    action['forward_vel'] = 0
                elif len(ShelfCenter) == 2:
                    print('Exiting the row')
                    robot_state = 'EXIT_ROW'    
                    action['rotational_vel'] = 0
                    action['forward_vel'] = 0
                elif len(ShelfCenter) == 0 and avg_center == 0:
                    robot_state = 'EXIT_ROW'  
                    print('Looking for exit')
                    action['rotational_vel'] = 0
                    action['forward_vel'] = 0
            elif robot_state == 'EXIT_ROW':
                print('Exiting the row')
                MoveForward(10)
                robot_state = 'STOPPING'



            elif robot_state == 'STOPPING':
                action['forward_vel'] = 0
                action['rotational_vel'] = 0
                print('Stopping')

            # Update velocities based on the robot state
            cv2.imshow('RobotView', RobotView)
            cv2.waitKey(1) # this is required to update the display
            SetSpeed(action['forward_vel'], action['rotational_vel'])
            warehouseBotSim.UpdateObjectPositions()

    except KeyboardInterrupt as e:
        # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
        warehouseBotSim.StopSimulator()