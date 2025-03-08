#!/usr/bin/python
import vision as vs
import time
import cv2
import numpy as np
from threading import Thread
import navigation as nav
from state_machine import MAIN_STATE, SUB_STATE, print_state
from mobility import MobilityModule
from RP2040 import I2C

paused_update = False
def SetSpeed(forward_vel, rotational_vel):
    if not paused_update:
      MobilityModule.Move(forward_vel*80, rotational_vel*-80)
    else:
        pass # do nothing if paused

def Turn45deg(Side):
    paused_update = True # pause the update loop
    if Side == 'left':
      print('Turning Left')
      MobilityModule.Move(0, -25)
      time.sleep(4)
      MobilityModule.Move(0, 0)
      time.sleep(1)
      paused_update = False # resume the update loop
    elif Side == 'right':
      print('Turning Right')
      MobilityModule.Move(0, 25)
      time.sleep(4)
      MobilityModule.Move(0, 0)
      time.sleep(1)
      paused_update = False # resume the update loop

def MoveForward(delay):
    paused_update = True # pause the update loop
    print('Moving Forward')
    MobilityModule.Move(80, 0)
    time.sleep(delay)
    paused_update = False # resume the update loop


def MoveBackward(delay):
    paused_update = True # pause the update loop
    print('Moving Forward')
    MobilityModule.Move(-80, 0)
    time.sleep(delay)
    paused_update = False # resume the update loop


# MAIN SCRIPT
action = {}
goal_position = {}
avoid_position = {}
found_row = False
target_aisle = 3
main_state = MAIN_STATE.INIT
task = SUB_STATE.FIND_AISLE



if __name__ == '__main__':
# Wrap everything in a try except case that catches KeyboardInterrupts. 
# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C
    try:
        vision = vs.VisionModule()
        navigation = nav.NavigationModule()
        FRAME_WIDTH = 820
        FRAME_HEIGHT = 616
        cam = vs.CamFrameGrabber(src=0, height=FRAME_WIDTH, width=FRAME_HEIGHT)
        cam.start()
        goal_bay_position = [90, 70, 40, 10] # bay positions in the row in cm
        while True: 

            imgRGB, imgHSV, RobotView = cam.getCurrentFrame()
            frame_id = cam.getFrameID()
            CenterCoord = vision.draw_crosshair(RobotView)

            WallRGB,  WallImgGray, WallMask,contoursWall1, GrayScale_Image = vision.findWall(imgHSV,imgRGB)
            ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
            avg_center, avg_bearing, avg_distance, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)
            
            contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            ShelfCenters = vision.GetContoursShelf(contoursShelf, RobotView, (0, 0, 255), "S", Draw=True)
            ShelfCenter, ShelfBearing = vision.GetInfoShelf(RobotView, ShelfCenters, imgRGB)
            fake_corners, away_corners, facing_corners,further_edge_corners = vision.ProcessShelfCorners(contoursShelf, RobotView, draw=True)
            furthest_point = vision.find_furthest_point(further_edge_corners,RobotView) # Process the furthest point from the away corners
            furthest_point_distance = vision.CalculateDistancePoints(furthest_point,RobotView)


            contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
            # Get the list of detected obstacles' centers and dimensions
            detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)
            ObsTest, ObsCenter = vision.GetInfoObject(RobotView, detected_obstacles, imgRGB)





            #region INIT
            if main_state == MAIN_STATE.INIT:
                #task = SUB_STATE.FIND_ENTRY #NEED TO CHANGE THIS
                task = SUB_STATE.FIND_ENTRY
                main_state = MAIN_STATE.FIND_ORDER

                # main_state = MAIN_STATE.TRANSIT_ORDER
                # task = SUB_STATE.FIND_PACIKINGBAY
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
                    print(len(LoadingBayCenter))
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
                        if avg_distance < 65: #goal_bay_position[3]:
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
                    MoveForward(1) # Hard code to move forward for 3 seconds to pick up the item
                    MoveBackward(1)
                    main_state = MAIN_STATE.TRANSIT_ORDER
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
                            MoveForward(2) # Hard code to move forward for 14 seconds to exit the aisle
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
                        action['rotational_vel'] = -0.3
                        action['forward_vel'] = 0
                    elif len(LoadingBayCenter) > 0:
                        # Img might be split into 2 parts by the obstacle
                        avg_LoadingBayBearing = sum(LoadingBayBearing) / len(LoadingBayBearing)  
                        action['rotational_vel'] = -0.3
                        action['forward_vel'] = 0
                        if abs(avg_LoadingBayBearing) < 20:
                            action['rotational_vel'] = 0
                            action['forward_vel'] = 0
                            task = SUB_STATE.MOVE_TO_PACKINGBAY

                elif task == SUB_STATE.MOVE_TO_PACKINGBAY:
                    if len(LoadingBayCenter) > 0:
                        avg_LoadingBayBearing = sum(LoadingBayBearing) / len(LoadingBayBearing)  
                        goal_position['range'] = 10/100  # Assume the distance to the packing Bay is 10cm
                        if avg_distance is not None:
                            goal_position['range'] = avg_distance /100
                            goal_position['bearing'] = avg_LoadingBayBearing / 100
                        elif avg_distance is None:
                            goal_position['bearing'] = avg_LoadingBayBearing / 100
                            goal_position['range'] = 10/100  # Assume the distance to the packing Bay is 10cm
                    elif len(LoadingBayCenter) == 0:
                        goal_position['range'] = avg_distance /100
                        goal_position['bearing'] = avg_bearing /100
                    elif len(LoadingBayCenter) == 0 and shape_count == 0:  # Nothing is found
                        action['rotational_vel'] = 0
                        action['forward_vel'] = 0
                        task = SUB_STATE.LOST
                    # if avg_distance < 45:
                    #     task = SUB_STATE.DROP_ITEM
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
            cam.DisplayFrame(frame_id, FPS=True, frame=RobotView, frame1 = WallMask) # Display the frame with the detected objects
            print_state(main_state, task)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            SetSpeed(action['forward_vel'], action['rotational_vel'])

    except KeyboardInterrupt as e:
        # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
        MobilityModule.Move(0, 0)