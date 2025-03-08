import cv2
import numpy as np
from math import cos,sin,pi,atan2,degrees
import time

from .Globals import *
#from threading import Thread
# MADE BY KELVIN LE, QUT EGB320 GROUP16 
# StudentID: n11429984
# Email: minhnguyen.le@qut.edu.au

class VisionOutput:
	# Basically a class where we can put whatever data we want
	def __init__(self, **kwargs):
		for key, value in kwargs.items():
			self.__setattr__(key, value)

class VisionModule:
	color_ranges = {
		'wall': (np.array([24, 25, 175]), np.array([39, 105, 255])),
		'floor': (np.array([0, 0, 0]), np.array([179, 255, 255])),
		'yellow': (np.array([29, 177, 244]), np.array([30, 251, 255])),
		'blue': (np.array([81, 0, 0]), np.array([116, 255, 255])),
		'green': (np.array([55, 79, 0]), np.array([70, 255, 255])),
		'orange1': (np.array([0, 100, 0]), np.array([20, 255, 255])),
		'orange2': (np.array([165, 100, 0]), np.array([180, 255, 255])),
		'black': (np.array([38,31, 45]), np.array([66, 121 , 88]))
	}
	
	#region Contour Processing 
	@classmethod
	def calculate_projection_transform(cls):
		# DIST_X, DIST_Z and TILT are defined in Globals.py

		# Precalculate Transformation Matrices and ground plane
		cls.camera_to_robot_rotate = np.array([
				[cos(TILT), 0, -sin(TILT), 0],
				[        0, 1,          0, 0],
				[sin(TILT), 0,  cos(TILT), 0],
				[        0, 0,          0, 1]])
		cls.camera_to_robot_translate = np.array([
				[1, 0, 0, DIST_X],
				[0, 1, 0,      0],
				[0, 0, 1, DIST_Z],
				[0, 0, 0,      1]])
		
		cls.camera_to_robot = np.matmul(cls.camera_to_robot_translate,cls.camera_to_robot_rotate)
		cls.robot_to_camera = np.linalg.inv(cls.camera_to_robot)
		#robot_to_camera_translate = np.linalg.inv(camera_to_robot_translate)
		cls.robot_to_camera_rotate = np.linalg.inv(cls.camera_to_robot_rotate)

		# Normal and point of the ground plane, relative to camera
		cls.normal_camera = np.matmul(cls.robot_to_camera_rotate , np.array([[0,0,1,1]]).T)[0:3, 0]
		cls.r_camera = np.matmul(cls.robot_to_camera , np.array([[0,0,0,1]]).T)[0:3, 0]

		cls.screen_normal_camera = np.array([-1,0,0])
		cls.screen_r_camera = np.array([0.5 / np.tan(FOV_HORIZONTAL/2),0,0])

	@classmethod
	def project_to_ground(cls,screen_coords):
		if screen_coords is None:
			return None
		elif len(screen_coords)<1:
			return screen_coords
		
		x = screen_coords[:, 0]
		y = screen_coords[:, 1]

		# Coordinates on a plane in front of the camera, relative to camera
		cx = -(x-SCREEN_WIDTH/2) / SCREEN_WIDTH
		cy = -(y-SCREEN_HEIGHT/2) / SCREEN_WIDTH
		cz = 0.5 / np.tan(FOV_HORIZONTAL/2) * np.ones(cx.shape)
		cpi = np.array([cz,-cx,cy])


		# coordinates on the ground, relative to camera
		cpo = np.dot(cls.normal_camera, cls.r_camera) / np.dot(cls.normal_camera, cpi) * cpi
		cpo = np.array([cpo[0,:], cpo[1,:], cpo[2,:], np.ones(cpo[0,:].shape)])

		# coodinates on the ground, relative to robot
		rpo = np.matmul(cls.camera_to_robot, cpo)
		return rpo[0:2,:].T

	@classmethod
	def project_to_screen(cls, ground_coords):
		if ground_coords is None:
			return None
		elif len(ground_coords)<1:
			return ground_coords
		
		# Coordinates on the ground plane, relative to robot
		rx = ground_coords[:, 0]
		ry = ground_coords[:, 1]
		rz = np.zeros(rx.shape)
		rw = np.ones(rx.shape)
		rpi = np.array([rx,ry,rz,rw])
		
		# Relative to camera
		cpi = np.matmul(cls.robot_to_camera, rpi)[0:3, :]


		# coordinates on the ground, relative to camera
		cpo = np.dot(cls.screen_normal_camera, cls.screen_r_camera) / np.dot(cls.screen_normal_camera, cpi) * cpi
		
		x =  cpo[1, :] * SCREEN_WIDTH + SCREEN_WIDTH/2
		y = -cpo[2, :] * SCREEN_WIDTH + SCREEN_HEIGHT/2
		

		
		return np.c_[x, y]


	@staticmethod
	def combine_contour_points(contours, exclude_horizontal_overlap=True):

		
		combined_contour = None
		xmax = -1
		xmin = SCREEN_WIDTH+1
		for cont in contours:

			
			if exclude_horizontal_overlap:
				bounds = cv2.boundingRect(cont) # left, top, width, height

				# only consider new contours strictly outside of what we have already seen
				# * this means that a contour between two already seen will be excluded! this should be pretty rare though *
				if bounds[0]+bounds[2] <= xmin:
					xmin = bounds[0]
					xmax = max(xmax, bounds[0]+bounds[2])
				elif bounds[0] >= xmax:
					xmax = bounds[0] + bounds[2]
					xmin = min(xmin, bounds[0])
				else:
					continue
			

			# npc is array of contour points along the edge (and around the outside) of the object
			npc = np.array(cont)
			npc = npc[:,0,:]

			npc = np.c_[npc, np.ones((npc.shape[0], 1), np.int32)] # new column : is real observed point
			
			# discard points on the edge of the screen and sort left to right (if two points are at same x prefer larger y (lower on the screen))
			# this is so later when we get unique elements the lower one is kept
			npc = npc[(npc[:,0] > 0) & (npc[:,0] < SCREEN_WIDTH-1), :]
			#npc = npc[(npc[:,1] > 0) & (npc[:,1] < SCREEN_HEIGHT-1) & (npc[:,0] > 0) & (npc[:,0] < SCREEN_WIDTH-1), :]
			npc = npc[np.argsort(npc[:, 0]), :]

			if npc.shape[0] < 3:
				continue

			# # append to comvined_contour
			if combined_contour is None:
				combined_contour = npc
			else:
				combined_contour = np.r_[combined_contour, npc]

		if combined_contour is not None:
			combined_contour = combined_contour[np.argsort(combined_contour[:,0])]
		return combined_contour

	@classmethod
	def handle_outer_contour(cls, combined_contour, value=SCREEN_HEIGHT-1):
		# *** ONLY does the OUTER ones !!
		if combined_contour is not None:
			# add a 0 point before and after so that the dist map counts blank spaces as close walls and will turn away from it and towards a shelf.
			# This will cause problems when trying to leave an aisle so don't call it in that case.
			if combined_contour[0,0] > 1: # any with 0 were removed so first being =1 is acceptable
				combined_contour = np.r_[[[combined_contour[0,0]-1, value, 0]], combined_contour]
			if combined_contour[-1,0] < SCREEN_WIDTH-2:# any with SCREEN_WIDTH-1 were removed so first being =SCREEN_WIDTH-2 is acceptable
				combined_contour = np.r_[combined_contour, [[combined_contour[-1,0]+1, value, 0]]]
		return combined_contour


	@classmethod
	def project_and_filter_contour(cls,contour_points):
		if contour_points is not None:
			# make sure it is sorted from left to right and only keep one point per column. (lower points preferred)	
			contour_points = contour_points[np.unique(contour_points[:, 0]+ 1.0-contour_points[:, 1]/SCREEN_HEIGHT, return_index=True)[1], :]

			# project contour onto the ground
			projection = cls.project_to_ground(contour_points)

			# discard points above the horizon
			mask = projection[:, 0] >= 0
			projection = projection[mask]
			contour_points = contour_points[mask, :]
			return contour_points, projection
		else:
			return None, None

	@staticmethod
	def get_dist_map(contour_points, projection):
		# distances of each point. However each point does not match 1 to 1 with pixels
		dist_real = np.sqrt(projection[:,0]**2 + projection[:,1]**2)
		
		# find which point matches to which pixel considering duplicates and skips
		dist_map = np.zeros(SCREEN_WIDTH, np.float32)
		real_points = np.zeros(SCREEN_WIDTH, np.float32)
		j = 0 # jth point
		for i in range(len(dist_map)): # ith pixel
			while j < len(contour_points)-1 and contour_points[j, 0] < i:
				j += 1

			# if we are not on the first point we can consider the previous point
			# and if out current point is past the current pixel we should look at the previous point
			if j > 0 and contour_points[j, 0] > i: 
				# we would take nearest, minimum, interpolate etc
				# take the one with max distance
				if dist_real[j-1] > dist_real[j]:
					dist_map[i] = dist_real[j-1]
					real_points[i] = contour_points[j-1, 2]
				else:
					dist_map[i] = dist_real[j]
					real_points[i] = contour_points[j, 2]
			else:
				dist_map[i] = dist_real[j]
				real_points[i] = contour_points[j, 2]
		return np.c_[dist_map, real_points]
	#endregion

	focal_length = 33 #mm # should be 35?
	real_circle_diameter = 70 #mm

 
	t1 = time.time()
	fps = 0

	#region Pipelines
	@classmethod
	def DebugPipeline(cls, draw=True):
		img, imgHSV, robotview = Specific.get_image()
		if img is None:
			print("Failed getting image")
			return None, None

		# Detect shelves in the HSV image
		contoursShelf, ShelfMask = cls.findShelf(imgHSV, 100, cv2.CHAIN_APPROX_NONE)
		contoursObstacle, ObstacleMask = cls.findObstacle(imgHSV, 200, cv2.CHAIN_APPROX_NONE)
		contoursLoadingArea, LoadingAreaMask = cls.findLoadingArea(imgHSV)
		contoursItem, ItemMask = cls.findItems(imgHSV)

		# Detect wall and marker within
		WallRGB,  WallImgGray, WallMask, contoursWall1 = cls.findWall(imgHSV,img)
		ContoursMarkers, mask1 = cls.findMarkers(WallImgGray, WallMask)
		avg_center, marker_bearing, marker_distance, aisle = cls.GetInfoMarkers(robotview, ContoursMarkers, draw=draw)


		detected_shelves = cls.ProcessContoursShelf(contoursShelf, robotview, (0,255,255),"Shelf", False)
		detected_obstacles = cls.ProcessContoursObject(contoursObstacle, robotview, (151,255,0), "Obstacle", False)
		detected_wall = cls.ProcessContoursObject(contoursWall1, robotview, (127,127,127), "Wall", False)

		ObsTest, ObsCenter = cls.GetInfoObject(robotview, detected_obstacles, img, draw)

		ShelfCorners = cls.ProcessShelfCorners(contoursShelf, robotview, draw=draw)

		return robotview, WallMask

	@classmethod
	def Pipeline(cls, draw=True):
		# Pipeline for use with navigation
		img, imgHSV, robotview = Specific.get_image()
		if img is None:
			print("Failed getting image")
			return None, None

		# Detect shelves in the HSV image
		contoursShelf, ShelfMask = cls.findShelf(imgHSV, 100, cv2.CHAIN_APPROX_NONE)
		contoursObstacle, ObstacleMask = cls.findObstacle(imgHSV, 200, cv2.CHAIN_APPROX_NONE)
		contoursLoadingArea, LoadingAreaMask = cls.findLoadingArea(imgHSV)
		contoursItem, ItemMask = cls.findItems(imgHSV)

		# Detect wall and marker within
		WallRGB,  WallImgGray, WallMask, contoursWall1 = cls.findWall(imgHSV,img)
		ContoursMarkers, mask1 = cls.findMarkers(WallImgGray, WallMask)
		avg_center, marker_bearing, marker_distance, aisle = cls.GetInfoMarkers(robotview, ContoursMarkers, draw=draw)


		detected_shelves = cls.ProcessContoursShelf(contoursShelf, robotview, (0,255,255),"Shelf", False)
		detected_obstacles = cls.ProcessContoursObject(contoursObstacle, robotview, (151,255,0), "Obstacle", False)
		detected_wall = cls.ProcessContoursObject(contoursWall1, robotview, (127,127,127), "Wall", False)

		ObsTest, ObsCenter = cls.GetInfoObject(robotview, detected_obstacles, img, draw)

		ShelfCorners = cls.ProcessShelfCorners(contoursShelf, robotview, draw=draw)

		return robotview, VisionOutput(
			aisle=aisle,
			marker_distance=marker_distance,
			marker_bearing=marker_bearing,
			contoursItem=contoursItem,
			contoursShelf=contoursShelf,
			contoursObstacle=contoursObstacle,
			detected_shelves=detected_shelves, 
			detected_wall=detected_wall,
			contoursLoadingArea=contoursLoadingArea,
			shelfCorners=ShelfCorners,
			obstacles=ObsTest,
			WallMask=WallMask
			)

	#endregion

	#region Image Utils
	@classmethod # Includes code to measure FPS and draw it onto the image frame
	def ExportImage(cls, WindowName, view, FPS=False):
		if FPS:
			fps = 1.0 / (time.time() - cls.t1)  # calculate frame rate
			cls.t1 = time.time()
			cv2.putText(view, f'{int(fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen
		
		
		cv2.imshow(WindowName, view)
		#cv2.setMouseCallback(WindowName, cls.PrintPixel, view)

	@staticmethod # Mainly for use while break'd in debug console
	def DebugShowIm(imgRGB, contours=None, markerpoint=None):
		imgRGB = imgRGB.copy()
		if contours is not None:
			imgRGB = cv2.drawContours(imgRGB, contours, -1, (0,0,255), 2)
		
		if markerpoint is not None:
			imgRGB = cv2.drawMarker(imgRGB, markerpoint, (0,255,255), cv2.MARKER_CROSS, 8, 2)
		
		cv2.imshow("Quick Debug", imgRGB)
		cv2.waitKey(1)
	#endregion

	#region Detection
	@classmethod
	def findBlack(cls, imgHSV):
		# Create masks for the orange color
		#HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
		BlackMask = cv2.inRange(imgHSV, cls.color_ranges['black'][0], cls.color_ranges['black'][1])
		#contoursBlack, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return BlackMask

	@classmethod
	def findItems(cls, imgHSV):
		# Create masks for the orange color
		ItemMask1 = cv2.inRange(imgHSV, cls.color_ranges['orange1'][0], cls.color_ranges['orange1'][1])
		ItemMask2 = cv2.inRange(imgHSV, cls.color_ranges['orange2'][0], cls.color_ranges['orange2'][1])
		ItemMask = ItemMask1 | ItemMask2  # Combine masks
		contoursItem, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return contoursItem, ItemMask

	@classmethod
	def findShelf(cls, imgHSV, area_threshold=100, chain=cv2.CHAIN_APPROX_SIMPLE):
		# Create a mask for the blue color range
		ShelfMask = cv2.inRange(imgHSV, cls.color_ranges['blue'][0], cls.color_ranges['blue'][1])
		
		# Find contours on the mask
		contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL,chain)
		
		# Filter out small contours by area
		contoursShelf = [cnt for cnt in contoursShelf if cv2.contourArea(cnt) > area_threshold]
		
		return contoursShelf, ShelfMask

	@classmethod
	def findLoadingArea(cls, imgHSV, area_threshold=100, chain=cv2.CHAIN_APPROX_SIMPLE):
		# Create a mask for the blue color range
		LoadingAreaMask = cv2.inRange(imgHSV, cls.color_ranges['yellow'][0], cls.color_ranges['yellow'][1])
		
		# Find contours on the mask
		contoursLoadingArea, _ = cv2.findContours(LoadingAreaMask, cv2.RETR_EXTERNAL, chain)

		# Filter out small contours by area
		contoursLoadingArea = [cnt for cnt in contoursLoadingArea if cv2.contourArea(cnt) > area_threshold]

		return contoursLoadingArea, LoadingAreaMask

	@classmethod
	def findObstacle(cls, imgHSV, area_threshold=100, chain=cv2.CHAIN_APPROX_SIMPLE):
		ObstacleMask = cv2.inRange(imgHSV, cls.color_ranges['green'][0], cls.color_ranges['green'][1])
		#ObstacleMask = cv2.morphologyEx(ObstacleMask, cv2.MORPH_DILATE, np.ones((20,20)))
		contoursObstacle, _ = cv2.findContours(ObstacleMask, cv2.RETR_EXTERNAL, chain)

		# Filter out small contours by area
		contoursObstacle = [cnt for cnt in contoursObstacle if cv2.contourArea(cnt) > area_threshold]

		return contoursObstacle, ObstacleMask
	
	# @classmethod
	# def findWall(cls, imgHSV, imgRGB):
	# 	# Create masks for the orange color (wall detection)
	# 	WallMask = cv2.inRange(imgHSV, cls.color_ranges['wall'][0], cls.color_ranges['wall'][1])
		
	# 	# Find contours in the mask
	# 	contoursWall1, _ = cv2.findContours(WallMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
	# 	# Check if any contours are found
	# 	if contoursWall1:
	# 		contoursWall1 = sorted(contoursWall1, key=cv2.contourArea, reverse=True)

	# 		# Create an empty mask and draw the convex hull on it
	# 		filledWallMask = np.zeros_like(WallMask)

			
	# 		# Find the 2 largest contours
	# 		for largest_contour in contoursWall1[:2]:
	# 			if cv2.contourArea(largest_contour) > 400: # some threshold
	# 				# Calculate the convex hull of the largest contour
	# 				#hull = cv2.convexHull(largest_contour)
					
	# 				cv2.drawContours(filledWallMask, [largest_contour], -1, (255), thickness=cv2.FILLED)
			
	# 		# Apply Gaussian blur to the filled mask
	# 		filledWallMask = cv2.GaussianBlur(filledWallMask, (9, 9), 2)
			
	# 		# Use the filled mask to extract the wall image
	# 		WallImg = cv2.bitwise_and(imgRGB, imgRGB, mask=filledWallMask)
			
	# 		# Convert the extracted image to grayscale
	# 		WallImgGray = cv2.cvtColor(WallImg, cv2.COLOR_BGR2GRAY)
	# 	else:
	# 		# No contours found, return original image and empty mask
	# 		WallImg = np.zeros_like(imgRGB)
	# 		WallImgGray = np.zeros_like(cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY))
	# 		filledWallMask = np.zeros_like(WallMask)

	# 	return WallImg, WallImgGray, filledWallMask, contoursWall1

	##
	@classmethod
	def findWall(cls, imgHSV, imgRGB):
		# Create masks for the orange color (wall detection)
		gray = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY)
		ret, WallMask = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)
		#WallMask = cv2.inRange(imgHSV, self.color_ranges['wall'][0], self.color_ranges['wall'][1])

		# Find contours in the mask
		contoursWall1, _ = cv2.findContours(WallMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		# Check if any contours are found
		if contoursWall1:
			# Find the largest contour
			largest_contour = max(contoursWall1, key=cv2.contourArea)
			
			# Calculate the convex hull of the largest contour
			#hull = cv2.convexHull(largest_contour)
			
			# Create an empty mask and draw the convex hull on it
			filledWallMask = np.zeros_like(WallMask)
			cv2.drawContours(filledWallMask, [largest_contour], -1, (255), thickness=cv2.FILLED)
			
			# Apply Gaussian blur to the filled mask
			filledWallMask = cv2.GaussianBlur(WallMask, (9, 9), 2)
			
			# Use the filled mask to extract the wall image
			WallImg = cv2.bitwise_and(imgRGB, imgRGB, mask=filledWallMask)
			
			# Convert the extracted image to grayscale
			WallImgGray = cv2.cvtColor(WallImg, cv2.COLOR_BGR2GRAY)
			largest_contour = np.array([largest_contour])
		else:
			# No contours found, return original image and empty mask
			largest_contour = contoursWall1
			WallImg = np.zeros_like(imgRGB)
			WallImgGray = np.zeros_like(cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY))
			filledWallMask = np.zeros_like(WallMask)

		return WallImg, WallImgGray, filledWallMask, largest_contour
	##

	@classmethod # Returns markers contours inside wall. Need to call findWall first
	def findMarkers(cls, WallImgGray, WallMask):
		# mask = cv2.inRange(WallRGB, cls.color_ranges['black'][0], cls.color_ranges['black'][1])
		_, mask = cv2.threshold(WallImgGray, 130, 255, cv2.THRESH_BINARY_INV)
		markers = cv2.bitwise_and(WallMask, mask)
		_, mask1 = cv2.threshold(markers, 130, 255, cv2.THRESH_BINARY)
		ContoursMarkers, _ = cv2.findContours(markers, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return ContoursMarkers, mask1
	
	@classmethod # Input: marker contours, Output: marker avg center, bearing, distance, count (aisle number). Would be nice to tell if any markers are blocked
	def GetInfoMarkers(cls, robotview, ContoursMarkers, draw=True):
		distance = []
		bearing = []
		centers = []
		radii = []
		obstructed = []
		for contours in ContoursMarkers:
			if cv2.contourArea(contours) > 100:
				(x, y), radius = cv2.minEnclosingCircle(contours)
				center = (int(x), int(y))
				circle_area = np.pi * (radius ** 2)
				contour_area = cv2.contourArea(contours)
				
				# Define the acceptable difference threshold
				# We need to see += half of the expected circle
				# (we will not see more than expected because of minEnclosingCircle - the circle will just be bigger)
				area_difference_threshold = circle_area*0.5  # You can adjust this value

				# Check if the difference between areas is within the threshold
				if abs(contour_area - circle_area) <= area_difference_threshold:
					if abs(contour_area - circle_area) >= 0.15 * circle_area:
						obstructed.append(True)
					else:
						obstructed.append(False)

					MarkerAngle = cls.GetBearing(x)
					MarkerDistance = cls.GetDistance(radius * 2, 70)
					if draw:
						cv2.circle(robotview, center, int(radius), (255, 255), 2)
						cv2.drawMarker(robotview, center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=5, thickness=2)
						cv2.putText(robotview, f"M", (int(x - 6), int(y + radius / 2)), 
									cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
					
					# Store the distance, bearing, and center
					distance.append(MarkerDistance)
					bearing.append(MarkerAngle)
					centers.append(center)
					radii.append(radius)

		# Calculate the average center if there are any centers
		if centers:
			avg_x = sum([c[0] for c in centers]) / len(centers)
			avg_y = sum([c[1] for c in centers]) / len(centers)
			avg_radius = sum(radii) / len(radii)
			avg_center = (int(avg_x), int(avg_y))
			avg_distance = sum(distance) / len(distance)
			avg_bearing = sum(bearing) / len(bearing)
			shape_count = len(centers)

			if abs(avg_bearing) > FOV_HORIZONTAL/2 * 0.75:
				# Too close to edge of screen
				# Don't trust the aisle number
				shape_count = 0
			
			if shape_count == 1:
				# Should ignore aisle number if it is obstructed at all
				# or even if its too close to a shelf
				if obstructed[0]:
					shape_count = 0
			elif shape_count == 2:
				# Too far apart
				if (np.array([(x-avg_center[0])**2+(y-avg_center[1])**2 for x, y in centers]) > (5 * avg_radius)**2).any():
					avg_center = None
					avg_bearing = None
					avg_distance = None
					shape_count = 0
				if (np.array([radius - avg_radius for radius in radii]) > (avg_radius)**2).any():
					avg_center = None
					avg_bearing = None
					avg_distance = None
					shape_count = 0
				if abs(centers[0][1] - centers[1][1]) > 25:
					shape_count = 0
			elif shape_count == 3:
				pass


			if draw:
				cv2.drawMarker(robotview, avg_center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=5, thickness=2)
				if avg_bearing is not None:
					cv2.putText(robotview, f"A: {int(degrees(avg_bearing))} deg", (int(avg_x), int(avg_y + 20)), 
								cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
					cv2.putText(robotview, f"D: {int(avg_distance)} cm", (int(avg_x), int(avg_y)),
								cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
				cv2.putText(robotview, f"{shape_count}", (int(avg_x - 10), int(avg_y)),
							cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 100), 2)
		else:
			avg_center = None  # If no centers were found, return None or a default value
			avg_bearing = None
			avg_distance = None
			shape_count = 0

		return avg_center, avg_bearing, avg_distance, shape_count

	
	@classmethod # Formerly GetContoursShelf. Input: Shelf contours. Output: shelf center (not distance/bearing)
	def ProcessContoursShelf(cls, contours, output, colour, text, Draw=True):
		detected = False
		centers = []
		radius = 0
		for contour in contours:
			if cv2.contourArea(contour) > 100: # was 1000 but jasper changed to be consistent with contour filter in Pipeline
				# Calculate the contour's center using moments
				M = cv2.moments(contour)
				if M['m00'] != 0:  # Avoid division by zero
					center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
				else:
					center = (0, 0)
				
				if Draw:
					# Draw the contour
					cv2.drawContours(output, [contour], -1, colour, 1)
					
					# Draw the text at the center of the contour
					cv2.putText(output, text, center, 
								cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
					
				centers.append(center)
				detected = True
				# If you want to calculate a bounding box

		if detected:
			return centers
		else:
			return None
		
	@classmethod # Formerly GetContoursObject. Input contours. Output: center, width and height
	def ProcessContoursObject(cls, contours, output, colour, text, Draw=True):
		detected_objects = []  # List to store detected object info
		
		for contour in contours:
			if cv2.contourArea(contour) > 50:
				x, y, width, height = cv2.boundingRect(contour)
				
				if Draw:
					cv2.rectangle(output, (x, y), (x + width, y + height), colour, 1)
					cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				
				# Calculate center
				x_center, y_center = x + width // 2, y + height // 2
				
				# Append this object's properties to the list
				detected_objects.append((x_center, y_center, height, width))
		
		# Return list of detected objects
		if detected_objects:
			return detected_objects
		else:
			return None
		
	@classmethod
	def GetInfoObject(cls, robotview, detected_obstacles, imgRGB, draw=True):
		distance = []
		bearing = []
		centers = []

		if detected_obstacles is not None:
			# Loop through each detected obstacle and process it
			for obstacle in detected_obstacles:
				x_ObstacleCenter, y_ObstacleCenter, ObHeight, ObWidth = obstacle
				
				# Calculate the obstacle's angle and distance
				ObstacleAngle = cls.GetBearing(x_ObstacleCenter) # in radians
				ObstacleDistance = cls.GetDistance(ObHeight, 140)/100 # in cm
				if True:#abs(ObstacleAngle) < 0.32: # Ignore obstacles close to the FOV edge
					distance.append(ObstacleDistance)
					bearing.append(ObstacleAngle)
					centers.append((x_ObstacleCenter, y_ObstacleCenter))
				
				# Add the angle and distance information to the image
				if draw:
					cv2.putText(robotview, f"A: {int(ObstacleAngle*180/pi)} deg", (int(x_ObstacleCenter), int(y_ObstacleCenter + ObHeight / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
					cv2.putText(robotview, f"D: {int(ObstacleDistance)*100} m", (int(x_ObstacleCenter), int(y_ObstacleCenter)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)

		# Return a list of [distance, bearing] pairs
		return [[d, b] for d, b in zip(distance, bearing)], centers

	#endregion

	#region Shelf Corners
	@staticmethod
	def is_edge_line(pt1, pt2, image_width, image_height):
		"""Check if the line touches the edges of the frame."""
		x1, y1 = pt1
		x2, y2 = pt2

		# If both points lie on the image boundary (edges), consider it an edge line
		if (
			(x1 == 0 or x1 == image_width-1 or
			 y1 == 0) and# or y1 == image_height-1) and
			(x2 == 0 or x2 == image_width-1 or
			 y2 == 0)# or y2 == image_height-1)
			):
			return True
		return False


	@staticmethod
	def get_line_angle_degrees(vec):
		# Calculate the angle of the line relative to the horizontal
		dx, dy = vec #Take the coordinates of the vector
		angle_radians = atan2(dy, dx) # Calculate the angle in radians
		angle_degrees = degrees(angle_radians) # Convert to degrees
		return angle_degrees


	@classmethod
	def ProcessShelfCorners(cls, contours, robotview, draw=True):
		lines = []  # To store the lines and their lengths
		image_height, image_width = robotview.shape[:2]  # Get image dimensions

		corners = []

		for contour in contours:
			epsilon = 5#0.01 * cv2.arcLength(contour, True)
			approx = cv2.approxPolyDP(contour, epsilon, True)

			# Only consider contours with at least 4 points
			if len(approx) >= 4:
				# Collect line segments and their lengths
				for i in range(len(approx)):
					pt0 = approx[(i - 1 + len(approx)) % len(approx)][0]
					pt1 = approx[i][0] # Point1 of the line
					pt2 = approx[(i + 1) % len(approx)][0]  # Wrap around to the first point

					# Use pt1 as the "origin" for these vectors
					v10 = pt0 - pt1
					v12 = pt2 - pt1
					
					# Get signed angle of each and check if they are vertical
					ang1 = cls.get_line_angle_degrees(v10)
					ang2 = cls.get_line_angle_degrees(v12)
					theta = abs(ang2-ang1)
					vert1 = abs(ang1) > 70 and abs(ang1) < 110
					vert2 = abs(ang2) > 70 and abs(ang2) < 110

					# Discard if line2 is pointing too far to the left or line1 is pointing too far to the right 
					# This would mean we're on the 'top' of the contour
					if abs(ang2) > 105 or abs(ang1) < 75:
						continue
					
					# Only Eliminate points that touch the edges of the image
					if cls.is_edge_line(pt1, pt1, image_width, image_height):
						continue  # Skip this point if it touches the edge

					
					
					# Append (pt1, pt2, length) to the list
					lines.append((pt1, pt2))
					if draw:
						cv2.line(robotview, tuple(pt0), tuple(pt1), (0, 255, 0) if vert1 else (255, 0, 0), 2)
						cv2.line(robotview, tuple(pt1), tuple(pt2), (0, 255, 0) if vert2 else (255, 0, 0), 2)

					
					corner_type = ''

					# Discard corner if both are vertical
					if vert1 and vert2:
						continue

					if (not vert1) and (not vert2):
						# Both horizontal
						
						# At least one line should extend upwards (-y) from the point
						if ang1 > 0 and ang2 > 0:
							continue

						corner_type = 'Facing'
						imp_ang = (ang1, ang2)
					else:
						# One Horizontal and one vertical
						if vert1:
							# Horizontal Line must extend downwards
							if ang2 < -20 and ang2 > -160:
								continue
							
							imp_ang = ang2

							# Classify using the vertical line dir
							if ang1 > 0 :
								corner_type = 'Fake'
							else:
								corner_type = 'Away'
						
						elif vert2:
							# Horizontal Line must extend downwards
							if ang1 < -20 and ang1 > -160:
								continue

							imp_ang = ang1

							# Classify using the vertical line dir
							if ang2 > 0 :
								corner_type = 'Fake'
							else:
								corner_type = 'Away'

					corners.append((pt1, corner_type, imp_ang))

		if draw:
			for point, corner_type, imp_ang in corners:
				if corner_type == 'Facing':
					cv2.circle(robotview, tuple(point), 5, (255, 255, 255), -1)  # Draw white circle for facing corners
				elif corner_type == 'Fake':
					cv2.circle(robotview, tuple(point), 5, (127, 127, 127), -1)  # Draw grey circle for fake corners
				elif corner_type == 'Away':
					cv2.circle(robotview, tuple(point), 5, (0, 0, 0), -1)  # Draw black circle for away corners

		return corners
	#endregion
	
	@classmethod
	def GetDistance(cls, width, real_width):
		return (cls.focal_length * real_width) / width
	
	@classmethod
	def GetBearing(cls, x_center): # RADIANS!!!
		offset_pixels = x_center - SCREEN_WIDTH/ 2
		return offset_pixels * FOV_HORIZONTAL / SCREEN_WIDTH

