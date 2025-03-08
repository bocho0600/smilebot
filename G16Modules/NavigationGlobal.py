import numpy as np, cv2
from enum import Enum
from math import cos, sin, pi
import time

from .Globals import *
from .Vision import VisionModule
# from .ItemCollection import ItemCollectionModule
from .MapProcessing import ArenaMap
from .PathProcess import PathProcess

STATE = Enum('STATE', [
	'LOST',
	'WANDER',
	'FIND_AISLE_FROM_OUTSIDE',
	'AISLE_DOWN',
	'AISLE_DOWN_BAY3',
	'FACE_BAY',
	'COLLECT_ITEM',
	'FACE_OUT',
	'WANDER_OUT',
	'FACE_PACKING',
	'APPROACH_PACKING',
	'ASCEND_PACKING',
	'DROP_ITEM',
	'DESCEND_PACKING',
	])

PHASE = Enum('PHASE', [
	'COLLECT',
	'DROPOFF'
	])


class NavigationModule:


	
	MAX_ROBOT_VEL = 0.13 # m/s
	ROTATIONAL_BIAS = 0.7 #tweak this parameter to be more or less aggressive with turns vs straight
	Kp = 5.0 # proportional term. beware if its too high we will try to go backwards for sharp turns
	MAX_ROBOT_ROT = pi/6 # rad/s
	RADIUS = 0.15 # how far to stay away from wall

	@classmethod
	def set_velocity(cls, fwd, rot):

		# Specific.set_velocity(fwd, rot)
		PathProcess.set_velocity(fwd, rot)


	#region state machine
	# For each state define a start and update function stored in this dict
	current_state = STATE.WANDER
	

	@classmethod
	def init(cls, initial_state,instruction):
		
		cls.target_aisle = int(instruction[0])
		cls.target_bay = int(instruction[1])
		cls.target_side = instruction[2]
		cls.target_height = int(instruction[3])
		cls.target_object = instruction[4]
		print(f"Aisle: {cls.target_aisle}, Bay: {cls.target_bay}, Side: {cls.target_side}")

		cls.shelf_length = 112 #cm
		cls.bay_width = cls.shelf_length / 4
		cls.target_bay_distance = cls.shelf_length - cls.bay_width/2 - cls.target_bay*cls.bay_width
		print(f"We want to be {cls.target_bay_distance} cm from aisle marker {cls.target_aisle}")
		

		cls.current_state = initial_state
		cls.t_now = time.time()

		cls.map = ArenaMap()
		cls.map_image = cls.map.draw_arena(512)

		cls.target_position = cls.map.marker_locations[cls.target_aisle-1] - np.array([cls.target_bay_distance/100, 0])

		PathProcess.Start()
	
	@classmethod
	def end(cls):
		PathProcess.End()
	


	# Call current update function then update the current state
	@classmethod
	def update(cls,debug_img,visout):
		t_last = cls.t_now
		cls.t_now = time.time()
		delta = cls.t_now - t_last

		image = None

		localized = cls.try_localize(visout)

		if cls.has_pose_estimate:
			if not localized:
				cls.pose_x, cls.pose_y, cls.pose_h = PathProcess.get_current_track()
				cls.dist_since_localization += (PathProcess.fwd + 0.2*PathProcess.rot)*delta


			# Correction here

			# Navigation
			point = np.array([cls.pose_x, cls.pose_y])
			diff = cls.target_position - point
			force = 10 * cls.map.getForce(point) + diff # getforce is shelf/wall avoidance + diff is force to the goal
			goal_error = np.arctan2(force[1], force[0]) - cls.pose_h
			rotational_vel = max(min(goal_error*cls.Kp, cls.MAX_ROBOT_ROT), -cls.MAX_ROBOT_ROT)
			forward_vel = cls.MAX_ROBOT_VEL * (1.0 - cls.ROTATIONAL_BIAS*abs(rotational_vel)/cls.MAX_ROBOT_ROT)

			#print(f"Force: {force[0]:.4f}, {force[1]:.4f}")

			if debug_img is not None:
				image = cls.map_image.copy()
				robotcolor = (0,255,0) if localized else (0,0,255)
				cv2.line(image, (int(cls.pose_x*256), int(cls.pose_y*256)), (int((cls.pose_x+0.1*cos(cls.pose_h))*256), int((cls.pose_y+0.1*cls.pose_h)*256)), robotcolor, cv2.MARKER_TRIANGLE_UP, 5)
				cv2.line(image, (int(cls.pose_x*256), int(cls.pose_y*256)), (int((cls.pose_x+0.025*force[0])*256), int((cls.pose_y+0.025*force[1])*256)), (255,0,255), cv2.MARKER_TRIANGLE_UP, 3)

				cv2.drawMarker(image, (256*cls.target_position).astype(np.int32), (0,0,0), cv2.MARKER_CROSS, 12)
				cv2.drawMarker(image, (int(cls.pose_x*256), int(cls.pose_y*256)), robotcolor, cv2.MARKER_TRIANGLE_UP, 20)

			cls.set_velocity(forward_vel, rotational_vel)

			if cls.dist_since_localization > 4.0:
				cls.we_are_lost()
		else:
			print("sad") # idk (use temp map)

		return debug_img, image
	

	has_pose_estimate = False
	dist_since_localization = -1
	
	pose_x = None
	pose_y = None
	pose_h = None

	@classmethod
	def current_position(cls):
		return np.array([cls.pose_x, cls.pose_y])

	@classmethod
	def we_are_lost(cls):
		print("WE ARE LOST !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
		cls.has_pose_estimate = False
		cls.dist_since_localization = -1
		
		cls.pose_x = None
		cls.pose_y = None
		cls.pose_h = None
		PathProcess.stop_integrating()

	@classmethod
	def localize(cls, x, y, heading):
		#print(f"We are at ({x:.2f}, {y:.2f})")
		x_center = x - DIST_X * cos(heading)
		y_center = y - DIST_X * sin(heading)

		cls.pose_x = x_center
		cls.pose_y = y_center
		cls.pose_h = heading
		PathProcess.localize(x_center, y_center, heading)

		cls.dist_since_localization = 0
		cls.has_pose_estimate = True

	@classmethod
	def try_localize(cls, visout):
		# Check if we have enough information to localize absolutely (i.e., marker or packing station unobstructed)
		# vis pipeline visout should give us processed shelf contours with lines and facing/fake/away points, sorted by x,
		# marker and obstacle info etc. We want to project shelf points to floor and be able to find normals!

		# 	If not, use dead reckoning (from pathprocess, it has more accurate integration of the "odometry")
		#	just update our position, or if we don't have an estimate, update our position in the temp map (new made up term :)

		# 	Check if we have enough information to correct a dead-reckonning
		#	based on shelf points or obstructed marker/packing station
		#	the only situations where we cant localize or correct are too close to a wall or shelf, or facing a wall

		# If we know where we are
		# 	and vision matches, locate obstacles, plan path and execute.
		# 		Don't even react just follow the path
		# 	but it doesn't match up
		# 		use local navigation to avoid obstacles.
		#		if time since localization gets too high, has_pose_estimate = false and we go back to using temp map
		# If we don't know where we are
		# 	wander (try and find the packing station)

		# Temp map is observed points, track important points using dead reckoning only (cant be bothered to correct within temp map)
		# More recently observed points have higher priority
		# Points expire after some time
		# points include shelf and obstacles
		# to include an obstacle on the map we need to have a decently good estimate of our own position
		# Time since localization or distance travelled since localization could be a good indicator of how confident we are in our current position esitimate.
		# (Assume all successful localizations are exactly correct!)
		# When we start we ought to be able to localize straight away off the packing station.
		# If its obstructed just use the temp map to go towards it until its not obstructed.
		# When navigating temp map, 2d potential field is used (?). We have points and normals, just avoid the points and ignore normals

		# How can we 'control' to the centre of the aisle or to face an item using PID or state space?
		# For centre of aisle we can just set the trajectory to the center and (somehow) state-space ourselves along the trajectory
		# In this case we should only recalculate the trajectory if we stray too far from it,
		# rather than every loop recalculating a new trajectory that starts from our current position. 
		# Doing that could work, I think I will find out because it is much more feasible to implement
		# For facing an item, there can be a special state that continues setting exponentially decaying rotating paths until we are facing it
		# In this case we can ignore most of the vision processing and definitely ignore localization (we won't have enough info)
		# The repeated paths set could be exponential or just a single segment that expires when we should be facing it
		
		# Just need to consider that the motors have a minimum speed and we could get stuck not quite facing it and not generating enough torque to face it
		# Maybe a minimum speed offset should be introduced in the mobility code, then assume linear between that and max.
		# (i.e. what is the minimum pwm value to get any actual movement (max with no movement, +1)).
		# Even then there could be a minimum real speed which we can expect to move
		# in that case we just have to make sure that whenever we really want to move, we make sure we move faster than that.
		# Whenever we want to move really slowly for some reason, just move minimum speed for less time. It will work
		

		def checkContour(cont):
			return cont is not None and len(cont) > 0

		if False: # it isn't reliable enough # checkContour(visout.contoursLoadingArea) and visout.aisle == 1:
			# We might be able to localize off loading area
			# Information: distance to marker, and bearing of BOTH edges of the loading area
			# Both are needed to make sure one is the part touching the wall. (cant use the corner in the middle of the floor)

			# Todo should also be checking if the packing station is obscured by a shelf (or obstacle)

			left, top, width, height = cv2.boundingRect(visout.contoursLoadingArea[0])
			bottom = top+height
			right = left+width

			if left > 0 and right < SCREEN_WIDTH-1 and top > 0 and bottom < SCREEN_HEIGHT-1:
				left_bearing = (left - SCREEN_WIDTH/2) * FOV_HORIZONTAL / SCREEN_WIDTH
				right_bearing = (right - SCREEN_WIDTH/2) * FOV_HORIZONTAL / SCREEN_WIDTH
				marker_bearing = visout.marker_bearing
				marker_offet = 0.062 # marker distance from wall, x or y not diagonal
				md = visout.marker_distance/100 * 1.06 # seemed to be a bit under idk why. 
				lw = 0.585 # loading area width
				lw_prime = np.sqrt((lw - marker_offet)**2 + marker_offet**2)
				alpha_offset = np.arcsin(marker_offet / lw_prime)


				# Choose the one closer in our view to the marker
				ld = marker_bearing - left_bearing
				rd = right_bearing - marker_bearing

				if rd < ld:
					# use right side
					theta = rd
					alpha_prime = pi - np.arcsin(np.sin(theta)*md/lw_prime)
					alpha = alpha_prime + alpha_offset
					phi = pi - theta - alpha
					x = marker_offet + md * np.cos(phi)
					y = marker_offet + md * np.sin(phi)
					heading = phi - pi - marker_bearing
				else:
					# use left side
					theta = ld
					alpha_prime = pi - np.arcsin(np.sin(theta)*md/lw_prime)
					alpha = alpha_prime + alpha_offset
					phi = pi - theta - alpha
					x = marker_offet + md * np.sin(phi)
					y = marker_offet + md * np.cos(phi)
					heading = -pi/2 - phi - marker_bearing

				if not np.isnan(x) and not np.isnan(y):
					cls.localize(x, y, heading,)
					return True
		
		elif visout.aisle > 0 and checkContour(visout.contoursShelf) and not checkContour(visout.contoursObstacle):
			# Marker, shelf and obstacle
			# We might be able to localize of marker+shelf angle
			# We will need to check that the marker is not obscured
			
			if visout.shelfCorners is not None and len(visout.shelfCorners) > 0 and len(visout.shelfCorners) <= 4:
				# We are facing down an aisle. Need to check that the two corners left and right of the marker are both 'away' and get their angles

				corner1 = None
				corner2 = None

				marker_pixel = (visout.marker_bearing * SCREEN_WIDTH/FOV_HORIZONTAL) + SCREEN_WIDTH/2
				md = visout.marker_distance / 100
				marker_bearing = visout.marker_bearing
				for i, corner in enumerate(reversed(visout.shelfCorners)):
					if corner[0][0] < marker_pixel:
						if corner[1] == 'Away':
							corner1 = corner
						break
				for i, corner in enumerate(visout.shelfCorners):
					if corner[0][0] > marker_pixel:
						if corner[1] == 'Away':
							corner2 = corner
						break
				
				if corner1 is not None and corner2 is not None:
					

					my_points = np.array([
						corner1[0], corner1[0] + 70*np.array([cos(corner1[2]*pi/180), sin(corner1[2]*pi/180)]),
				  		corner2[0], corner2[0] + 70*np.array([cos(corner2[2]*pi/180), sin(corner2[2]*pi/180)])])
				
					ppoints = VisionModule.project_to_ground(my_points)

					theta1 = marker_bearing - (corner1[0][0]-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH
					theta2 = (corner2[0][0]-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH - marker_bearing

					def angle_between(vec1, vec2):
						dot = (vec1*vec2).sum()
						l1 = np.sqrt((vec1*vec1).sum())
						l2 = np.sqrt((vec2*vec2).sum())
						return np.arccos(dot/l1/l2)

					if abs(ppoints[0][0] - ppoints[2][0]) < 0.2: # protect against alse localization when the close point is detected as "away"
						phi1 = angle_between(-ppoints[0], ppoints[1]-ppoints[0])
						phi2 = angle_between(-ppoints[2], ppoints[3]-ppoints[2])

						#print(f"can localize off corner angles: {corner1[2]:.1f} and {corner2[2]:.1f}... Got error {(phi1+phi2-theta1-theta2)*180/pi:.1f} degrees.")

						f1 = theta1-phi1
						f2 = phi2-theta2
						f = (f1 + f2) / 2
						x = cls.map.marker_locations[visout.aisle-1][0] - md*np.cos(f)
						y = cls.map.marker_locations[visout.aisle-1][1] - md*np.sin(f)
						heading = f - marker_bearing

						cls.localize(x, y, heading)
						return True
	
		return False

	