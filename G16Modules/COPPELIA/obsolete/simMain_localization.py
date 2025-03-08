#!/usr/bin/python

# import the  bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *
import numpy as np, cv2
import time
from math import cos, sin, pi
from pfilter import pfilter as pf 
from scipy.stats import norm, gamma, uniform
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from mapProcessing import ArenaMap

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

def dynamics_fn(x, delta):
	xp = np.array(x)
	xp[:,0] = xp[:,0] + delta*xp[:,3]*np.cos(xp[:,2])
	xp[:,1] = xp[:,1] + delta*xp[:,3]*np.sin(xp[:,2])
	xp[:,2] = xp[:,2] + delta*xp[:,4]
	return xp

def w_fun(x):
	sigma = 3.5
	ss = sigma**2
	return np.exp(-ss * x**2)

def weight_fn(states, observation, delta):
	# states: particle possible states N * D
	# observation: real observation 1 * H
	
	# return weights (how likely each state is to yeild the real observation) N * 1

	observation = np.c_[observation, np.zeros((observation.shape[0],1)), np.ones((observation.shape[0],1))]
	weights = []
	for state in states:
		t = state[2]
		t_rotate = np.array([
				[cos(t), 0, -sin(t), 0],
				[        0, 1,    0, 0],
				[sin(t), 0,  cos(t), 0],
				[        0, 0,    0, 1]])
		t_obs = np.matmul(observation, np.transpose(t_rotate))[:, 0:2] + state[0:2]
		closests = []
		wf = []
		for i in range(t_obs.shape[0]):
			seg, dist = arena.closestSegment(t_obs[i,:], "shelf", state[0:2])
			closests.append(dist)
			wf.append(w_fun(dist))
		
		weights.append(np.prod(wf))


	return weights

def observation_fn(img):
	n_points = 20
	scale = SCREEN_WIDTH / n_points
	squash_view = cv2.resize(img, (n_points, SCREEN_HEIGHT))
	contour, mask = findFloor(squash_view)
	
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3)))
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3)))

	detection_img = mask - np.roll(mask, 1, 0)
	coords = np.transpose(np.nonzero(detection_img==255))
	observation = coords[coords[:,1].argsort()]
	observation[:,1] = observation[:,1] * scale
	observation = np.fliplr(observation) # x and y swap
	return project_point_to_ground(observation)
	

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

# Initialise pfilter 
columns = ["x", "y", "angle", "speed", "ang_vel"]
prior_fn = pf.independent_sample([uniform(0, 2).rvs, uniform(0, 2).rvs, lambda x: np.zeros(x), lambda x: np.zeros(x), lambda x: np.zeros(x)])

arena = ArenaMap()

PF = pf.ParticleFilter(
			prior_fn=prior_fn, 
			n_particles=200,
			dynamics_fn=dynamics_fn,
			noise_fn=lambda x,delta: 
						pf.gaussian_noise(x, sigmas=[0.025, 0.025, 0.0, 0.0, 0.0]),
			weight_fn=weight_fn,
			resample_proportion=0.05,
			column_names = columns)


# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C

	try:
		# Start Simulator
		packerBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
		packerBotSim.StartSimulator()
		packerBotSim.SetCameraPose(0.1, 0.1, 0)

		ANGLE_H = packerBotSim.horizontalViewAngle
		ANGLE_V = packerBotSim.verticalViewAngle
		DIST_X = 0#packerBotSim.robotParameters.cameraDistanceFromRobotCenter
		DIST_Z = 0.0752#packerBotSim.robotParameters.cameraHeightFromFloor
		# tilt = packerBotSim.robotParameters.cameraTilt
		TILT = 1.5 * 3.1415926535 / 180
		# tilt is meant to be 0 but it is slightly off in simulator

		# Precalculate Transformation Matrices and ground plane
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


		t_now = time.time()
		# plt.ion()
		# fig, ax = plt.subplots()
		# fig.subplots_adjust(left=0.25, bottom=0.25)
		# axsig = fig.add_axes([0.25, 0.1, 0.65, 0.03])
		# sigma_slider = Slider(
		# 	ax=axsig,
		# 	label='Sigma',
		# 	valmin=0.0,
		# 	valmax=0.5,
		# 	valinit=0.01,
		# )

		# sigmas = []
		# dispersions = []

		while True:
			t_last = t_now
			t_now = time.time()
			delta = t_now-t_last

			resolution, img = packerBotSim.GetCameraImage()

			img = convert_image(img)
			img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT))
			

			contour, mask = findFloor(img)

			res = cv2.bitwise_and(img,img,mask = mask)
			projected_image = np.zeros(shape=res.shape, dtype=np.uint8)
			
			large_contour = []
			for cont in contour:
				if cv2.contourArea(cont) > 500:
					# # npc is array of contour points along the edge (and around the outside) of the object
					# npc = np.array(cont)
					# npc = npc[:,0,:]
					

					# # draw
					# res = cv2.polylines(res, [npc], True, (0, 0, 255), 1) # draw

					# # project contour onto the ground
					# npc_project = (project_point_to_ground(npc)*200).astype(np.int32)

					# # move to the center of the image (roughly)
					# npc_project = np.array([npc_project[:, 0], npc_project[:, 1]+SCREEN_HEIGHT//2]).T

					# # draw
					# projected_image = cv2.polylines(projected_image, [npc_project], True, (255,0,0), 2)
					
					large_contour.append(contour)
					
			
			

			observation = observation_fn(img)
			# observation = np.array([0.3,0.3,0,0,0])
			PF.update(observation, delta=delta)
			#print(f"State: {PF.map_state[0:2]}, error {np.sqrt((np.array(PF.map_state - observation) ** 2).sum()):.4f} dispersion {PF.dispersion}")
			#sigmas.append(sigma_slider.val)
			#dispersions.append(PF.dispersion)
			# ax.clear()
			# ax.scatter(PF.particles[:, 0], PF.particles[:, 1])

			# ax.set(xlim=(0, 2),
			# 	ylim=(0, 2))
			
			for i in range(observation.shape[0]):
				projected_image = cv2.drawMarker(projected_image, (int(observation[i,0]*200), int(observation[i,1]*200+SCREEN_HEIGHT//2)), (255,0,0), cv2.MARKER_CROSS,4)
			
			map_image = arena.draw_arena(512)
			for i in range(PF.particles.shape[0]):
				map_image = cv2.drawMarker(map_image, (PF.particles[i, 0:2]*256).astype(np.int16), (0,0,255), cv2.MARKER_CROSS,4)
			

			#cv2.imshow("Raw", img)
			cv2.imshow("Debug", res)
			cv2.imshow("Projection", projected_image)
			cv2.imshow("Localization", map_image)
			cv2.waitKey(1)

			# plt.show(block=True)

			packerBotSim.SetTargetVelocities(0, 0.0)  # forward velocity, rotation
			packerBotSim.UpdateObjectPositions() # needs to be called once at the end of the main code loop

			

			# time.sleep(0.1)

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		# packerBotSim.StopSimulator()
		print("Stop")
		# fig, ax = plt.subplots()
		# ax.scatter(sigmas, dispersions)
		
		# plt.show(block = True)