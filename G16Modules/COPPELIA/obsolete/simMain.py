#!/usr/bin/python

# import the  bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *
import numpy as np, cv2
import time
from math import cos, sin

def nothing(x):
    pass

def convert_image(img):
	img = np.reshape((np.array(img).astype(np.uint8)), (480,640,3))
	return cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)

color_ranges = {
            'floor': (np.array([0, 0, 80]), np.array([0, 0, 103])),
            'wall': (np.array([0, 0, 146]), np.array([30, 1, 255])),
            'blue': (np.array([3, 171, 54]), np.array([3, 175, 112])),
            'black': (np.array([0, 0, 0]), np.array([0, 0, 0])),
            'yellow': (np.array([99, 216, 130]), np.array([99, 217, 187])),
            'green': (np.array([40, 90, 0]), np.array([70, 255, 180])),
            'orange1': (np.array([5, 150, 150]), np.array([20, 255, 255])),
            'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
      }

def findFloor(img):
	imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	FloorMask = cv2.inRange(imgHSV, color_ranges['floor'][0], color_ranges['floor'][1])
	contoursFloor, _ = cv2.findContours(FloorMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	return contoursFloor, FloorMask

def project_point_to_ground(screen_coords):
	x = screen_coords[:, 0]
	y = screen_coords[:, 1]

	
	# Coordinates on a plane in front of the camera, relative to camera
	cx = -(x-SCREEN_WIDTH/2) / SCREEN_WIDTH
	cy = -(y-SCREEN_HEIGHT/2) / SCREEN_WIDTH
	cz = 1 / ANGLE_H * np.ones(cx.shape)
	cpi = np.array([cz,-cx,cy])


	# coordinates on the ground, relative to camera
	cpo = np.dot(normal_camera, r_camera) / np.dot(normal_camera, cpi) * cpi
	cpo = np.array([cpo[0,:], cpo[1,:], cpo[2,:], np.ones(cpo[0,:].shape)])

	# coodinates on the ground, relative to robot
	rpo = np.matmul(camera_to_robot, cpo)
	return rpo[0:2,:].T

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240



# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.bayContents= np.random.random_integers(0,5,(6,4,3)) # Random item in each bay
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl # specify a bowl in the bay in shelf 0 
# with x,y coords (3,1) (zero-indexed). Items are {bowl,mug,bottle,soccer,rubiks,cereal}.

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'	# specify if using differential (currently omni is not supported)



# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C

	try:
		packerBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
		packerBotSim.StartSimulator()
		packerBotSim.SetCameraPose(0.1, 0.1, 0)

		ANGLE_H = packerBotSim.horizontalViewAngle
		ANGLE_V = packerBotSim.verticalViewAngle
		DIST_X = packerBotSim.robotParameters.cameraDistanceFromRobotCenter
		DIST_Z = packerBotSim.robotParameters.cameraHeightFromFloor
		# tilt = packerBotSim.robotParameters.cameraTilt
		TILT = 1.5 * 3.1415926535 / 180
		# tilt is meant to be 0 but it is slightly off in simulator

		# Transformation Matrices
		camera_to_robot_rotate = np.array([
				[cos(TILT), 0, -sin(TILT), 0],
				[        0, 1,          0, 0],
				[sin(TILT), 0,  cos(TILT), 0],
				[        0, 0,          0, 1]])
		camera_to_robot_translate = np.array([
				[1, 0, 0, DIST_X],
				[0, 1, 0,      0],
				[0, 0, 1, DIST_Z],
				[0, 0, 0,      1]])
		
		camera_to_robot = np.matmul(camera_to_robot_translate,camera_to_robot_rotate)
		
		robot_to_camera = np.linalg.inv(camera_to_robot)
		robot_to_camera_translate = np.linalg.inv(camera_to_robot_translate)
		robot_to_camera_rotate = np.linalg.inv(camera_to_robot_rotate)

		# Normal and point of the ground plane, relative to camera
		normal_camera = np.matmul(robot_to_camera_rotate , np.array([[0,0,1,1]]).T)[0:3, 0]
		r_camera = np.matmul(robot_to_camera , np.array([[0,0,0,1]]).T)[0:3, 0]

		current_frame_time = time.time()

		cv2.namedWindow("Raw")
		cv2.namedWindow("Mask")

		while True:
			resolution, img = packerBotSim.GetCameraImage()

			img = convert_image(img)
			img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT))
			#img = cv2.medianBlur(img, 5)

			contour, mask = findFloor(img)

			res = cv2.bitwise_and(img,img,mask = mask)
			marker_image = np.zeros(shape=res.shape, dtype=np.uint8)
			
			for cont in contour:
				if cv2.contourArea(cont) > 500:
					# npc is array of contour points along the edge (and around the outside) of the object
					npc = np.array(cont)
					npc = npc[:,0,:]
					

					# draw
					res = cv2.polylines(res, [npc], True, (0, 0, 255), 1) # draw

					# project contour onto the ground
					npc_project = (project_point_to_ground(npc)*200).astype(np.int32)

					# move to the center of the image (roughly)
					npc_project = np.array([npc_project[:, 0], npc_project[:, 1]+SCREEN_HEIGHT//2]).T

					# draw
					marker_image = cv2.polylines(marker_image, [npc_project], True, (255,0,0), 2)
					
					
			cv2.imshow("Raw", img)
			cv2.imshow("Mask", res)
			cv2.imshow("markers", marker_image)
			cv2.waitKey(1)
			
			packerBotSim.SetTargetVelocities(0, 0)  # forward velocity, rotation
			packerBotSim.UpdateObjectPositions() # needs to be called once at the end of the main code loop

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		packerBotSim.StopSimulator()