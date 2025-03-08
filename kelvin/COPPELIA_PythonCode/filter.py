#!/usr/bin/python
import vision as vs
import time
import cv2
# import the bot module - this will include math, time, numpy (as np), and vrep python modules
from warehousebot_lib import *
import navigation as nav

sceneParameters = SceneParameters()

# Starting contents of the bays [shelf,X,Y]. Set to -1 to leave empty.
sceneParameters.bayContents = np.random.randint(0, 6, sceneParameters.bayContents.shape)
sceneParameters.bayContents[0, 3, 1] = warehouseObjects.bowl
sceneParameters.bayContents[1, 1, 2] = warehouseObjects.mug
sceneParameters.bayContents[2, 3, 1] = warehouseObjects.bottle
sceneParameters.bayContents[3, 1, 2] = warehouseObjects.soccer
sceneParameters.bayContents[4, 2, 0] = warehouseObjects.rubiks
sceneParameters.bayContents[5, 0, 1] = warehouseObjects.cereal

sceneParameters.obstacle0_StartingPosition = [-0.5, 0]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = [0.5, -0.5]   # starting position of obstacle 1 [x, y] (in metres)
sceneParameters.obstacle2_StartingPosition = -1  # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'  # specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0    # minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25   # maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1      # specifies how good your drive system is from 0 to 1


# Define nothing function for trackbar callback
def nothing(x):
    pass

# MAIN SCRIPT
if __name__ == '__main__':
    try:
        # Initialize CoppeliaSim with warehouseBotSim
        warehouseBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
        warehouseBotSim.StartSimulator()

        # Initialize Vision Module
        vision = vs.VisionModule()

        # Define bay positions
        goal_bay_position = [0.875, 0.625, 0.375, 0.1]  # bay positions in the row
        target_row = 0

        # Create windows for image and mask
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)  
        cv2.resizeWindow('image', 800, 600)
        cv2.namedWindow('Mask')

        # Create trackbars for HSV color filtering
        cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
        cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
        cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
        cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
        cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
        cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

        # Set default values for Max HSV trackbars
        cv2.setTrackbarPos('HMax', 'image', 179)
        cv2.setTrackbarPos('SMax', 'image', 255)
        cv2.setTrackbarPos('VMax', 'image', 255)

        # Initialize HSV min/max values
        hMin = sMin = vMin = hMax = sMax = vMax = 0
        phMin = psMin = pvMin = phMax = psMax = pvMax = 0

        while True:
            # Capture frame from warehouseBotSim
            frame, hsv, RobotView = warehouseBotSim.Capturing()

            # Get current positions of HSV trackbars
            hMin = cv2.getTrackbarPos('HMin', 'image')
            sMin = cv2.getTrackbarPos('SMin', 'image')
            vMin = cv2.getTrackbarPos('VMin', 'image')
            hMax = cv2.getTrackbarPos('HMax', 'image')
            sMax = cv2.getTrackbarPos('SMax', 'image')
            vMax = cv2.getTrackbarPos('VMax', 'image')

            # Set minimum and maximum HSV values
            lower = np.array([hMin, sMin, vMin])
            upper = np.array([hMax, sMax, vMax])

            # Apply mask and filter the image
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            # Print if there's a change in HSV values
            if (phMin != hMin or psMin != sMin or pvMin != vMin or phMax != hMax or psMax != sMax or pvMax != vMax):
                print(f"(hMin = {hMin}, sMin = {sMin}, vMin = {vMin}), (hMax = {hMax}, sMax = {sMax}, vMax = {vMax})")
                phMin = hMin
                psMin = sMin
                pvMin = vMin
                phMax = hMax
                psMax = sMax
                pvMax = vMax

            # Display result and mask
            cv2.imshow('image', result)
            cv2.imshow('Mask', mask)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    finally:
        # Ensure simulator and windows are stopped and closed properly
        warehouseBotSim.StopSimulator()
        cv2.destroyAllWindows()
