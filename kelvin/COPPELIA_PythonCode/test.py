#!/usr/bin/python
import vision as vs
import time
import cv2
# import the  bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *
import navigation as nav
from state_machine import MAIN_STATE, SUB_STATE, print_state

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
#sceneParameters.obstacle1_StartingPosition = [0.5,-0.5]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
#sceneParameters.obstacle1_StartingPosition = [-0.52,0.27]

sceneParameters.obstacle0_StartingPosition = -1 
sceneParameters.obstacle1_StartingPosition = -1  # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = -1 
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
    warehouseBotSim.SetTargetVelocities(0.2, 0)
    time.sleep(delay)
    paused_update = False # resume the update loop


def MoveBackward(delay):
    paused_update = True # pause the update loop
    print('Moving Forward')
    warehouseBotSim.SetTargetVelocities(-0.2, 0)
    time.sleep(delay)
    paused_update = False # resume the update loop


# MAIN SCRIPT
action = {}
goal_position = {}
avoid_position = {}
found_row = False
target_aisle = 3
main_state = MAIN_STATE.INIT
#task = SUB_STATE.FIND_AISLE
tas = SUB_STATE.FIND_ENTRY



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
            fake_corners, away_corners, facing_corners, further_edge_corners = vision.ProcessShelfCorners(contoursShelf, RobotView, draw=True)
            furthest_point = vision.find_furthest_point(further_edge_corners,RobotView) # Process the furthest point from the away corners
            furthest_point_distance = vision.CalculateDistancePoints(furthest_point,RobotView)


            contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
            detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)
            ObsTest, ObsBearing = vision.GetInfoObject(RobotView, detected_obstacles, imgRGB)





            #region INIT
            if main_state == MAIN_STATE.INIT:
                #task = SUB_STATE.FIND_ENTRY #NEED TO CHANGE THIS
                task = SUB_STATE.FIND_ENTRY
                main_state = MAIN_STATE.FIND_ORDER

                # main_state = MAIN_STATE.TRANSIT_ORDER
                # task = SUB_STATE.FIND_EXIT
                action['forward_vel'] = 0
                action['rotational_vel'] = 0
            #endregion

            #region FIND_ORDER
            if main_state == MAIN_STATE.FIND_ORDER:
                found_row = False
                if task == SUB_STATE.FIND_ENTRY:
                    action['rotational_vel'] = -0.3
                    action['forward_vel'] = 0
                    if len(ShelfCenter) == 1:
                        
                        avg_shelf_bearing = sum(ShelfBearing) / len(ShelfBearing) 
                        print(avg_shelf_bearing)
                        if avg_shelf_bearing > -25 and avg_shelf_bearing < 0: 
                            task = SUB_STATE.MOVE_TO_ENTRY
                
                elif task == SUB_STATE.MOVE_TO_ENTRY:
                    if furthest_point_distance is not None:
                        #print(furthest_point_distance)
                        print(vision.GetBearing(furthest_point[0], RobotView))
                    if furthest_point_distance is not None:
                        goal_position['range'] = furthest_point_distance/100
                        goal_position['bearing'] = vision.GetBearing(furthest_point[0], RobotView) /100
                        action = navigation.calculate_goal_velocities(goal_position, ObsTest)
                        if furthest_point_distance < 20:
                            task = SUB_STATE.FIND_AISLE
                            action['forward_vel'] = 0
                            action['rotational_vel'] = 0
                    else:
                        task = SUB_STATE.FIND_AISLE
                        action['forward_vel'] = 0
                        action['rotational_vel'] = 0
                elif task == SUB_STATE.FIND_AISLE:
                    
                    contoursLoadingBay, LoadingBayMask = vision.findLoadingArea(imgHSV)
                    LoadingBayCenters = vision.GetContoursShelf(contoursLoadingBay, RobotView, (0, 255, 0), "L", Draw=True)
                    LoadingBayCenter, LoadingBayBearing = vision.GetInfoShelf(RobotView, LoadingBayCenters, imgRGB)
                    if shape_count == 0 or len(LoadingBayCenter) > 0: # If no markers detected, keep turning
                        action['rotational_vel'] = -0.3
                        action['forward_vel'] = 0
                    elif shape_count != 0 and len(LoadingBayCenter) == 0:  # If markers detected
                        if abs(avg_bearing) > 20: # If the robot is not facing the markers
                            action['rotational_vel'] = -0.3
                            action['forward_vel'] = 0
                        elif abs(avg_bearing) < 20: # If the robot is facing the markers
                            action['rotational_vel'] = -0.1
                            action['forward_vel'] = 0
                            if abs(avg_bearing) < 10:
                                action['rotational_vel'] = 0
                                action['forward_vel'] = 0
                                if shape_count == target_aisle and len(LoadingBayCenter) == 0: # If the robot is facing the markers
                                    found_row =  True
                                    print('Aisle found')
                                else:
                                    task = SUB_STATE.SWAP_AISLE
                    if shape_count == target_aisle and len(LoadingBayCenter) == 0:
                        found_row = True

                    if found_row:
                        print('Target Aisle found!')
                        task = SUB_STATE.FIND_BAY
                        action['rotational_vel'] = 0
                        action['forward_vel'] = 0

                elif task == SUB_STATE.FIND_BAY:
                    found_row = False
                    if shape_count > 0:
                        found_row = True
                        goal_position['range'] = avg_distance /100
                        goal_position['bearing'] = avg_bearing /100
                        if avg_distance < goal_bay_position[2]:
                            # REMEMBER TO UNCOMMENT THIS!!!!
                            main_state = MAIN_STATE.TAKE_ORDER
                            task = SUB_STATE.FIND_ITEM
                            # main_state = MAIN_STATE.TRANSIT_ORDER
                            # task = SUB_STATE.FIND_EXIT
                            action['forward_vel'] = 0
                            action['rotational_vel'] = 0
                    if found_row:
                        action = navigation.calculate_goal_velocities(goal_position, ObsTest)
                    else:
                        action['forward_vel'] = 0
                        action['rotational_vel'] = 0
                        #task = SUB_STATE.LOST
                        task = SUB_STATE.FIND_AISLE
            #endregion

            #region TAKE_ORDER
            elif main_state == MAIN_STATE.TAKE_ORDER:
                if task == SUB_STATE.FIND_ITEM:
                    contoursItem, ItemMask = vision.findItems(imgHSV)
                    detected_items = vision.GetContoursObject(contoursItem, RobotView, (255, 255, 255), "Obs", Draw=True)
                    Turn45deg('left')
                    action['rotational_vel'] = 0
                    action['forward_vel'] = 0
                    task = SUB_STATE.PICK_ITEM
                elif task == SUB_STATE.PICK_ITEM:
                    # Do something to pick up the item from the selected level
                    MoveForward(2) # Hard code to move forward for 3 seconds to pick up the item
                    main_state = MAIN_STATE.TRANSIT_ORDER
                    MoveBackward(3)
                    task = SUB_STATE.FIND_EXIT
                    action['forward_vel'] = 0
                    action['rotational_vel'] = 0
            #endregion

            #region TRANSIT_ORDER
            elif main_state == MAIN_STATE.TRANSIT_ORDER:
                contoursLoadingBay, LoadingBayMask = vision.findLoadingArea(imgHSV)
                LoadingBayCenters = vision.GetContoursShelf(contoursLoadingBay, RobotView, (0, 255, 0), "L", Draw=True)
                LoadingBayCenter, LoadingBayBearing = vision.GetInfoShelf(RobotView, LoadingBayCenters, imgRGB)
                if task == SUB_STATE.FIND_EXIT:
                    if len(LoadingBayCenter) > 0:
                        shape_count = 0
                    if len(ShelfCenter) < 2 :
                        action['rotational_vel'] = 0.3
                        action['forward_vel'] = 0
                    elif len(ShelfCenter) == 2 and (shape_count == 0):
                        # action['rotational_vel'] = 0
                        # action['forward_vel'] = 0
                        task = SUB_STATE.EXIT_ROW

                    # if no shelf and no markers, in this case is when the robot is facing towards the exit
                    elif len(ShelfCenter) == 0 and shape_count == 0:
                        print("stuck@")
                        task = SUB_STATE.EXIT_ROW
                        action['rotational_vel'] = 0
                        action['forward_vel'] = 0

                elif task == SUB_STATE.EXIT_ROW:
                    if len(ShelfCenter) == 2:
                        goal_position['range'] = 10/100  # Assume the distance to the exit is 10cm
                        goal_position['bearing'] = ((ShelfBearing[0] + ShelfBearing[1]) / 100)*2
                        action = navigation.calculate_goal_velocities(goal_position, ObsTest)
                    # elif len(ShelfCenter) == 1 and shape_count == 0:
                    #     task = SUB_STATE.FIND_EXIT
                    elif len(ShelfCenter) == 0:
                        if shape_count == 0: # If can't see any loading bay when facing outside the aisle
                            action['rotational_vel'] = 0.1
                            action['forward_vel'] = 0
                            if len(LoadingBayCenter) > 0:
                                area = cv2.contourArea(contoursLoadingBay[0])
                                print(area)
                                if area > 10000:
                                    task = SUB_STATE.FIND_PACIKINGBAY
                                else:
                                    task = SUB_STATE.EXIT_PACKINGBAY
                            else :
                                task = SUB_STATE.FIND_PACIKINGBAY
                            
                        elif shape_count == 1: # If see the loading bay
                            task = SUB_STATE.MOVE_TO_PACKINGBAY
                            action['rotational_vel'] = 0
                            action['forward_vel'] = 0
                    elif len(ShelfCenter) == 1:
                        avg_shelf_bearing = sum(ShelfBearing) / len(ShelfBearing)  
                        if avg_shelf_bearing < 0:
                            goal_position['bearing'] = ((32 + avg_shelf_bearing) / 100)*2
                        elif avg_shelf_bearing > 0:
                            goal_position['bearing'] = ((-32 + avg_shelf_bearing) / 100)*2
                        goal_position['range'] = 10/100  # Assume the distance to the exit is 10cm
                        action = navigation.calculate_goal_velocities(goal_position, ObsTest)

                elif task == SUB_STATE.FIND_PACIKINGBAY:
                    if len(LoadingBayCenter) == 0: # if no loading bay detected
                        action['rotational_vel'] = -0.2
                        action['forward_vel'] = 0
                    elif len(LoadingBayCenter) > 0:
                        # Img might be split into 2 parts by the obstacle
                        avg_LoadingBayBearing = sum(LoadingBayBearing) / len(LoadingBayBearing)  
                        action['rotational_vel'] = -0.2
                        action['forward_vel'] = 0
                        if abs(avg_LoadingBayBearing) < 20:
                            action['rotational_vel'] = 0
                            action['forward_vel'] = 0
                            task = SUB_STATE.MOVE_TO_PACKINGBAY

                elif task == SUB_STATE.MOVE_TO_PACKINGBAY:
                    if len(LoadingBayCenter) > 0:
                        avg_LoadingBayBearing = sum(LoadingBayBearing) / len(LoadingBayBearing)  
                        #goal_position['range'] = 10/100  # Assume the distance to the packing Bay is 10cm
                        goal_position['range'] = avg_distance /100
                        goal_position['bearing'] = avg_LoadingBayBearing / 100
                    elif len(LoadingBayCenter) == 0:
                        goal_position['range'] = avg_distance /100
                        goal_position['bearing'] = avg_bearing /100
                    elif len(LoadingBayCenter) == 0 and shape_count == 0:  # Nothing is found
                        action['rotational_vel'] = 0
                        action['forward_vel'] = 0
                        task = SUB_STATE.LOST
                    if avg_distance < 45:
                        task = SUB_STATE.DROP_ITEM
                    action = navigation.calculate_goal_velocities(goal_position, ObsTest)        

                elif task == SUB_STATE.DROP_ITEM:
                    action['rotational_vel'] = 0
                    action['forward_vel'] = 0
                    time.sleep(3)
                    # DO SOMETHING TO DROP THE ITEM
                    task = SUB_STATE.EXIT_PACKINGBAY
                
                elif task == SUB_STATE.EXIT_PACKINGBAY:
                    MoveBackward(10)
                    main_state = MAIN_STATE.INIT
                

                elif task == SUB_STATE.LOST:
                    action['rotational_vel'] = 0
                    action['forward_vel'] = 0
                    


            # Update velocities baprint_statesed on the robot state
            cv2.imshow('RobotView', RobotView)
            print_state(main_state, task)
            cv2.waitKey(1) # this is required to update the display
            SetSpeed(action['forward_vel'], action['rotational_vel'])
            warehouseBotSim.UpdateObjectPositions()

    except KeyboardInterrupt as e:
        # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
        warehouseBotSim.StopSimulator()