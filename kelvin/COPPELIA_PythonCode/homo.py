import vision as vs
import time
import cv2
# import the bot module - this will include math, time, numpy (as np), and vrep python modules
from warehousebot_lib import *
import navigation as nav
import numpy as np
sceneParameters = SceneParameters()

# Starting contents of the bays [shelf,X,Y]. Set to -1 to leave empty.
sceneParameters.bayContents = np.random.randint(0, 6, sceneParameters.bayContents.shape)
sceneParameters.bayContents[0, 3, 1] = warehouseObjects.bowl
sceneParameters.bayContents[1, 1, 2] = warehouseObjects.mug
sceneParameters.bayContents[2, 3, 1] = warehouseObjects.bottle
sceneParameters.bayContents[3, 1, 2] = warehouseObjects.soccer
sceneParameters.bayContents[4, 2, 0] = warehouseObjects.rubiks
sceneParameters.bayContents[5, 0, 1] = warehouseObjects.cereal


sceneParameters.obstacle1_StartingPosition = -1  # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = -1 
# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'  # specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0    # minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25   # maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1      # specifies how good your drive system is from 0 to 1

# Camera Parameters
robotParameters.cameraOrientation = 'landscape'
robotParameters.cameraDistanceFromRobotCenter = 0.1
robotParameters.cameraHeightFromFloor = 0.15
robotParameters.cameraTilt = 0.0

# Vision Processing Parameters
robotParameters.maxItemDetectionDistance = 1  # max distance to detect items
robotParameters.maxPackingBayDetectionDistance = 2.5  # max distance to detect packing bay
robotParameters.maxObstacleDetectionDistance = 1.5  # max distance to detect obstacles
robotParameters.maxRowMarkerDetectionDistance = 2.5  # max distance to detect row markers

# Collector Parameters
robotParameters.collectorQuality = 1
robotParameters.maxCollectDistance = 0.15

robotParameters.sync = False  # This parameter forces the simulation into sync

def onClick(event, x, y, flags, params):
	global image_points
	if event == cv2.EVENT_LBUTTONDOWN:
		print(x,y)
		if not found_homography:
			image_points.append([x,y])
		else:
			print('Predicted point')
			pred_point = cv2.perspectiveTransform(np.float32([[x,y]]).reshape(-1,1,2), M)
			print(pred_point)

# define position of points in the ground plane relative to the robot
ground_points = np.float32([[-18,20],[18,47],[-18,100],[18,100]]) # cm
# click the points in the image (order is important) to populate the image points
image_points = []

# Initialize CoppeliaSim with warehouseBotSim
warehouseBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
warehouseBotSim.StartSimulator()
frame, hsv, RobotView = warehouseBotSim.Capturing()


# reduce size to speed up	
#frame = cv2.rotate(frame, cv2.ROTATE_180)

# display
cv2.imshow("Image", frame)
cv2.setMouseCallback("Image", onClick)

# click four points and then fit the homography
found_homography = False
while True:
	cv2.waitKey(1)
	if len(image_points) == 4:
		if not found_homography:
			M, mask = cv2.findHomography(np.float32(image_points).reshape(-1,1,2), ground_points.reshape(-1,1,2))
			found_homography = True
			print("found homography")
			print(M)
			
# shutdown
cap.close()
cv2.destroyAllWindows()