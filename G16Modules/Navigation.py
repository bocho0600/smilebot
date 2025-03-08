import numpy as np, cv2
from enum import Enum
from math import cos, sin, pi
import math
import time

from .Globals import *
from .Vision import VisionModule
# from .ItemCollection import ItemCollectionModule
from .PathProcess import PathProcess

STATE = Enum('STATE', [
	'LOST',
	'LOST_OUTSIDE_AISLE',
	'LOST_INSIDE_AISLE',
	'AISLE_DOWN',
	'BLIND_MOVE', # This one is more of an "action" than a state
	'COLLECT_ITEM',
	'AISLE_OUT',
	'APPROACH_PACKING',
	'DROP_ITEM',
	'VEGETABLE',
	'AVOID_MOVE'
	])

PHASE = Enum('PHASE', [
	'COLLECT',
	'DROPOFF'
	])


class NavigationModule:
	
	if is_simulator:
		MAX_ROBOT_VEL = 0.13 # m/s
		ROTATIONAL_BIAS = 0.83 #tweak this parameter to be more or less aggressive with turns vs straight
		Kp1 = 2.4 # proportional term. beware if its too high we will try to go backwards for sharp turns
		Kp2 = 2.4 
		MAX_ROBOT_ROT = pi/5 # rad/s
		RADIUS = 0.13 # how far to stay away from wall
		MIN_ROTATION = 1.0
		ITEM_BIAS = 0.0
	else:
		MAX_ROBOT_VEL = 0.19 # m/s
		ROTATIONAL_BIAS = 0.83 #tweak this parameter to be more or less aggressive with turns vs straight
		Kp1 = 3.5 # FOR DRIVING
		Kp2 = 1.8 # FOR FACING
		MAX_ROBOT_ROT = pi/4 # rad/s
		RADIUS = 0.20 # how far to stay away from wall
		MIN_ROTATION = 1.0
		ITEM_BIAS = 0.0

	@classmethod
	def set_velocity(cls, fwd, rot, delta=0, fwdlen=None, rotlen=None):
		#cls.forced_avoidance_timer_update(fwd, delta)

		# Specific.set_velocity(fwd, rot)
		if fwdlen is None and rotlen is None:
			PathProcess.set_velocity(fwd, rot)
		else:
			PathProcess.new_path([(fwd, rot, fwdlen, rotlen)])


	#region state machine
	# For each state define a start and update function stored in this dict
	state_callbacks = {}
	current_state = STATE.LOST
	current_phase = PHASE.COLLECT

	@classmethod
	def process_instruction(cls):

		instruction = cls.instructions[cls.current_instruction]

		if not hasattr(cls, "target_aisle") or cls.target_aisle is None:
			cls.last_target_aisle = 3
		else:
			cls.last_target_aisle = 1#cls.target_aisle

		cls.target_aisle = int(instruction[0])
		cls.target_bay = int(instruction[1])
		cls.target_side = instruction[2]
		cls.target_height = int(instruction[3])
		cls.target_object = instruction[4]
		print(f"Aisle: {cls.target_aisle}, Bay: {cls.target_bay}, Side: {cls.target_side}")

		cls.shelf_length = 112 #cm
		cls.bay_width = cls.shelf_length / 4
		cls.target_bay_distance = cls.shelf_length - cls.bay_width/2 - cls.target_bay*cls.bay_width + 8
		print(f"We want to be {cls.target_bay_distance} cm from aisle marker {cls.target_aisle}")
		print(f"Height {cls.target_height}")

	@classmethod
	def init(cls, initial_state, instructions, starting_instruction=0):

		cls.instructions = instructions
		cls.current_instruction = starting_instruction
		cls.process_instruction()

		cls.state_callbacks[STATE.LOST] = (cls.LOST_start, cls.LOST_update)
		cls.state_callbacks[STATE.LOST_OUTSIDE_AISLE] = (cls.LOST_OUTSIDE_AISLE_start, cls.LOST_OUTSIDE_AISLE_update)
		cls.state_callbacks[STATE.LOST_INSIDE_AISLE] = (cls.LOST_INSIDE_AISLE_start, cls.LOST_INSIDE_AISLE_update)
		cls.state_callbacks[STATE.AISLE_DOWN] = (cls.AISLE_DOWN_start, cls.AISLE_DOWN_update)
		cls.state_callbacks[STATE.BLIND_MOVE] = (cls.BLIND_MOVE_start, cls.BLIND_MOVE_update)
		cls.state_callbacks[STATE.COLLECT_ITEM] = (cls.COLLECT_ITEM_start, cls.COLLECT_ITEM_update)
		cls.state_callbacks[STATE.AISLE_OUT] = (cls.AISLE_OUT_start, cls.AISLE_OUT_update)
		cls.state_callbacks[STATE.APPROACH_PACKING] = (cls.APPROACH_PACKING_start, cls.APPROACH_PACKING_update)
		cls.state_callbacks[STATE.DROP_ITEM] = (cls.DROP_ITEM_start, cls.DROP_ITEM_update)
		cls.state_callbacks[STATE.VEGETABLE] = (lambda *args: None, lambda *args: None)
		cls.state_callbacks[STATE.AVOID_MOVE] =  (cls.AVOID_MOVE_start, cls.AVOID_MOVE_update)

		cls.current_state = initial_state
		cls.t_now = time.time()
		cls.call_current_start()
		#cls.forced_avoidance_start()

		PathProcess.RADIUS = cls.RADIUS
		PathProcess.Start()
	
	@classmethod
	def end(cls):
		PathProcess.End()
	

	# Call the start function for the current state
	@classmethod
	def call_current_start(cls):
		print(f"State is now {cls.current_state.name}")
		return cls.state_callbacks[cls.current_state][0]()
	
	# Call the update function for the current state
	@classmethod
	def call_current_update(cls, delta,*args):
		return cls.state_callbacks[cls.current_state][1](delta,*args)

	# Call current update function then update the current state
	@classmethod
	def update(cls,*args):
		t_last = cls.t_now
		cls.t_now = time.time()
		delta = cls.t_now - t_last

		new_state, debug_img = cls.call_current_update(delta,*args)
		if new_state != cls.current_state:
			cls.current_state = new_state
			cls.call_current_start()

		return debug_img, None
	#endregion

	#region Utility Functions
	
	@staticmethod
	def checkContour(cont):
		return cont is not None and len(cont) > 0

	@staticmethod
	def clip_deg_fov(angle, FOV):
		if angle <= 0:
			angle = 0
		elif angle >= FOV:
			angle = FOV
		return angle   

	@staticmethod
	def deg_to_rad(deg):
		return math.radians(deg)

	@staticmethod
	def rad_to_deg(rad):
		return math.degrees(rad)

	@classmethod
	def compute_attractor_field(self, goal_deg):
		#CAMERA_FOV = 66
		attraction_field = np.zeros(CAMERA_FOV + 1)
		attraction_field[goal_deg] = 1
		gradient = 1 / 30
		for angle in range(0, int((CAMERA_FOV / 2 + 1))):
			attraction_field[self.clip_deg_fov(goal_deg - angle, CAMERA_FOV)] = 1 - angle * gradient
			attraction_field[self.clip_deg_fov(goal_deg + angle, CAMERA_FOV)] = 1 - angle * gradient
		return attraction_field

	@classmethod
	def compute_repulsive_field(cls, obstacles):
		WORKER_WIDTH_SCALE = 0.12  # m
		#CAMERA_FOV = 66
		
		repulsive_field = np.zeros(CAMERA_FOV + 1)
		
		if obstacles:
			for obs in obstacles:
				obs_range = obs[0]
				obs_bearing = obs[1]

				

				# only consider obstacles that are less than 1 metre away
				if obs_range < 1:
					# if obs_bearing > 0.7 * FOV_HORIZONTAL:
					# 	obs_range /= 2

					obs_deg = int(cls.rad_to_deg(obs_bearing) + CAMERA_FOV / 2)
					
					obs_width_rad = 2 * math.atan(WORKER_WIDTH_SCALE / obs_range)
					obs_width_deg = int(cls.rad_to_deg(obs_width_rad))
					
					obs_effect = max(0, 1 - min(1, obs_range - WORKER_WIDTH_SCALE * 2))

					repulsive_field[obs_deg] = obs_effect
					
					for angle in range(1, obs_width_deg + 1):
						repulsive_field[cls.clip_deg_fov(obs_deg - angle, CAMERA_FOV)] = max(
							repulsive_field[cls.clip_deg_fov(obs_deg - angle, CAMERA_FOV)], obs_effect)
						repulsive_field[cls.clip_deg_fov(obs_deg + angle, CAMERA_FOV)] = max(
							repulsive_field[cls.clip_deg_fov(obs_deg + angle, CAMERA_FOV)], obs_effect)

		return repulsive_field


	@classmethod
	def move_into_path(cls, target_bearing, debug_img, obstacles):
		# Todo implement this as an funciton to do all the shelf/obstacle contour point processing and avoidance
		# We might want to move towards the marker if visible, or track the marker from where we last saw it??
		# Or an esimated entry point

		def calculate_fwd_rot(goal_error):
			rotational_vel = max(min(goal_error*cls.Kp1, cls.MAX_ROBOT_ROT), -cls.MAX_ROBOT_ROT)
			forward_vel = cls.MAX_ROBOT_VEL * (1.0 - cls.ROTATIONAL_BIAS*abs(rotational_vel)/cls.MAX_ROBOT_ROT)
			return forward_vel, rotational_vel 

		# if debug_img is not None:
		# 	cv2.drawContours(debug_img, contours, -1, (0,0,255), 1)

		# points = VisionModule.combine_contour_points(contours, exclude_horizontal_overlap=False)
		# if handle_outer:
		# 	points = VisionModule.handle_outer_contour(points, handle_outer_value)
		# points, projected_floor = VisionModule.project_and_filter_contour(points)


		# if points is not None and points.shape[0] > 3:
		# 	dist_map = VisionModule.get_dist_map(points, projected_floor) # dist map column 0 is dist, column 1 is real point
		# else:
		# 	dist_map = np.ones((SCREEN_WIDTH, 2), np.float64)


		goal_deg = int(cls.clip_deg_fov(cls.rad_to_deg(target_bearing) + CAMERA_FOV / 2, CAMERA_FOV))
		
		# Compute both attractive and repulsive field maps
		attractive_field = cls.compute_attractor_field(goal_deg)
		repulsive_field = cls.compute_repulsive_field(obstacles)
		
		potential_field = np.maximum(0, attractive_field - repulsive_field)
		if debug_img is not None:
			debug_img = cv2.polylines(debug_img, [np.array([np.array(range(0, CAMERA_FOV+1))*SCREEN_WIDTH/CAMERA_FOV, SCREEN_HEIGHT - potential_field/4 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (255, 255, 0), 1) # draw
		

		# Find heading angle by finding the index of the max value in the residual field
		heading_angle = np.argmax(potential_field)
		goal_error = cls.deg_to_rad(heading_angle) - cls.deg_to_rad(CAMERA_FOV / 2)
	
		# Balance the amount of forward velocity vs rotational velocity if the robot needs to turn
		fwd, rot = calculate_fwd_rot(goal_error)

		# Target
		# If target is none go longest
		

		# if debug_img is not None:
		# 	debug_img = cv2.drawContours(debug_img, contours, -1, (0,0,255),2)
		# 	debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map[:,0]/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
		# 	debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (255, 255, 0), 1) # draw
				
		# Only can occur when there is no target
		# if abs(potential_map.min() - potential_map.max()) < 0.01:
		# 	if not force_forward:
		# 		# No safe path: move backwards
		# 		#cls.forced_avoidance_start() #reset FA
		# 		forward_vel = -cls.MAX_ROBOT_VEL/2
		# 		rotational_vel = 0
		# 		path_distance = 0
		# 	else:
		# 		if type(target_bearing) == str and target_bearing == 'longest':
		# 			goal_error = 0
		# 		else:
		# 			goal_error = target_bearing
		# 		path_distance = None
		# 		forward_vel, rotational_vel = calculate_fwd_rot(goal_error)

		if debug_img is not None:
			debug_img = cv2.drawMarker(debug_img, (int(goal_error*SCREEN_WIDTH/FOV_HORIZONTAL+SCREEN_WIDTH/2), SCREEN_HEIGHT//2), (0,255,255), cv2.MARKER_STAR, 10)
		# else:
		# 	# Move into highest potential

		# 	goal_error = (potential_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * FOV_HORIZONTAL
		# 	path_distance = max(dist_map[:,0])
		# 	forward_vel, rotational_vel = calculate_fwd_rot(goal_error)

		# 	if debug_img is not None:
		# 		debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - potential_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw
		# 		debug_img = cv2.drawMarker(debug_img, (potential_map.argmax(), int(SCREEN_HEIGHT - potential_map.max()/2 * SCREEN_HEIGHT)), (0,255,255), cv2.MARKER_STAR, 10)
			

		return fwd, rot #, path_distance


	#endregion
	
	#region State Callbacks

	@classmethod
	def LOST_start(cls):
		Specific.leds(0b100)
		print("Lost!")
		cls.is_inside_aisle = True
		PathProcess.new_path([(0, cls.MAX_ROBOT_ROT, None, 2*pi)])
	
	@classmethod
	def LOST_update(cls, delta, debug_img, visout):
		# turn 360
		# if we at any point see no shelves, -> LOST_OUTSIDE_AISLE
		# if not, -> LOST_INSIDE_AISLE
		# optionally calculate a (m/s to pwm factor)
		# also if we are in dropoff, try to remember where the packing station is. 

		if visout.detected_shelves is None:
			cls.is_inside_aisle = False
			return STATE.LOST_OUTSIDE_AISLE, debug_img
	
		if PathProcess.completed:
			return STATE.LOST_INSIDE_AISLE, debug_img


		return STATE.LOST, debug_img



	@classmethod
	def LOST_OUTSIDE_AISLE_start(cls):
		Specific.leds(0b100)
		# if we're going to aisle, find (estimate) entry point
		
		
		cls.loa_stage = -1
		
	
	@classmethod
	def LOST_OUTSIDE_AISLE_update(cls, delta, debug_img, visout):
		# if we're going to aisle, find (estimate) entry point
		# if not cls.at_entry_point and visout.aisle == cls.target_aisle and not cls.checkContour(visout.contoursLoadingArea):
		# 	return STATE.AISLE_DOWN, debug_img
		
		if cls.loa_stage == -1:
			# Decide which stage to start at
			if cls.checkContour(visout.contoursShelf):
				cls.loa_stage = 0
				if cls.target_aisle == 1 or (cls.target_aisle == 2 and cls.last_target_aisle >= 2):
					cls.set_velocity(0, -cls.MAX_ROBOT_ROT)
				elif cls.target_aisle == 3 or (cls.target_aisle == 2 and cls.last_target_aisle < 2):
					cls.set_velocity(0, cls.MAX_ROBOT_ROT)
			else:
				cls.loa_stage = 1
				if cls.target_aisle == 1 or (cls.target_aisle == 2 and cls.last_target_aisle >= 2):
					cls.set_velocity(0, cls.MAX_ROBOT_ROT)
				elif cls.target_aisle == 3 or (cls.target_aisle == 2 and cls.last_target_aisle < 2):
					cls.set_velocity(0, -cls.MAX_ROBOT_ROT)

		if cls.loa_stage == 0:
			# Until we can't see shelf
			if not cls.checkContour(visout.contoursShelf):
				cls.loa_stage = 1
				if cls.target_aisle == 1 or (cls.target_aisle == 2 and cls.last_target_aisle >= 2):
					cls.set_velocity(0, cls.MAX_ROBOT_ROT)
				elif cls.target_aisle == 3 or (cls.target_aisle == 2 and cls.last_target_aisle < 2):
					cls.set_velocity(0, -cls.MAX_ROBOT_ROT)
		
		if cls.loa_stage == 1:
			# Until we can see shelf
			if cls.checkContour(visout.contoursShelf):
				cls.loa_stage = 2
				cls.set_velocity(0,0)

		if cls.loa_stage == 2:
			# Until we are at entry point i.e. a location where we can see the CORRECT aisle marker and hopefully not crash into the side of the shelf


			def get_left_side(cont):
					x,y,w,h = cv2.boundingRect(cont.astype(np.int32))
					return x
			def get_right_side(cont):
					x,y,w,h = cv2.boundingRect(cont.astype(np.int32))
					return x+w



			if not cls.checkContour(visout.contoursShelf):
				cls.loa_stage = -1
				return STATE.LOST_OUTSIDE_AISLE, debug_img
			
			away_corners = [c for c in visout.shelfCorners if c[1] == 'Away']


			if cls.target_aisle == 1 or (cls.target_aisle == 2 and cls.last_target_aisle >= 2):
				obstacles = visout.obstacles
				if len(away_corners) > 0:
					leftmost = min(away_corners, key=lambda c: c[0][0])
					distance = np.linalg.norm(VisionModule.project_to_ground(np.array([leftmost[0]])))
					for c in visout.shelfCorners:
						if c[0][1] > leftmost[0][1] and c[1] == "Away":
							p = VisionModule.project_to_ground(np.array([c[0]]))
							o_range = np.linalg.norm(p)
							o_bearing = np.arctan2(p[0,1], p[0,0])
							obstacles.append([o_range, o_bearing])
				else:
					distance = None


				
				bearing = (get_left_side(min(visout.contoursShelf, key=get_left_side))-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH
				fwd, rot = cls.move_into_path(bearing, debug_img, obstacles)
			elif cls.target_aisle == 3 or (cls.target_aisle == 2 and cls.last_target_aisle < 2):
				obstacles = visout.obstacles
				if len(away_corners) > 0:
					rightmost = max(away_corners, key=lambda c: c[0][0])
					distance = np.linalg.norm(VisionModule.project_to_ground(np.array([rightmost[0]])))
					for c in visout.shelfCorners:
						if c[0][1] > rightmost[0][1] and c[1] == "Away":
							p = VisionModule.project_to_ground(np.array([c[0]]))
							o_range = np.linalg.norm(p)
							o_bearing = np.arctan2(p[0,1], p[0,0])
							obstacles.append([o_range, o_bearing])
				else:
					distance = None
				
				if cls.checkContour(visout.contoursLoadingArea):
					loadingArea = max(visout.contoursLoadingArea, key=cv2.contourArea)
					cx = get_right_side(loadingArea)
					la_bearing = (cx - SCREEN_WIDTH/2) * FOV_HORIZONTAL / SCREEN_WIDTH 
					la_range = 0.4
					obstacles.append([la_range, la_bearing])
					


				bearing = (get_right_side(max(visout.contoursShelf, key=get_right_side))-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH
				fwd, rot = cls.move_into_path(bearing, debug_img, obstacles)


			print(f"Distance :{distance}")

			# This is the main section we need to tweak to not crash into the shelf
			# Consider using the corner angle to inform the stopping distance (we have that info in visout!)
			if distance is not None and ((cls.target_aisle == 1 and distance <= 0.46) or\
				(cls.target_aisle == 2 and distance <= 1.25) or\
				(cls.target_aisle == 3 and ((cls.last_target_aisle >= 2 and distance <= 0.55) or
											(cls.last_target_aisle < 2 and distance <= 0.46)))):
				if visout.detected_shelves[0][0] > SCREEN_WIDTH/2:
					# shelf is on the right
					cls.aisle_dir = 0
					cls.set_velocity(0, cls.MAX_ROBOT_ROT)
				else:
					# shelf is on the left
					cls.aisle_dir = 1
					cls.set_velocity(0, -cls.MAX_ROBOT_ROT)
				print(f"At entry point ({distance:.2f})")
				cls.loa_stage = 3
			else:
				cls.set_velocity(fwd, rot, delta)
	
		if cls.loa_stage == 3:
			if not cls.checkContour(visout.contoursLoadingArea) and visout.marker_bearing is not None:
				cls.set_velocity(0,0)
				time.sleep(0.5)
				return STATE.AISLE_DOWN, debug_img
			elif visout.detected_shelves is not None and len(visout.detected_shelves) >= 2:
				if cls.aisle_dir == 0:
					# shelf is on the right
					cls.set_velocity(0, cls.MAX_ROBOT_ROT * 0.7)
				else:
					# shelf is on the left
					cls.set_velocity(0, -cls.MAX_ROBOT_ROT * 0.7)



		return STATE.LOST_OUTSIDE_AISLE, debug_img

	@classmethod
	def LOST_INSIDE_AISLE_start(cls):
		Specific.leds(0b100)
	
	@classmethod
	def LOST_INSIDE_AISLE_update(cls, delta, debug_img, visout):
		if cls.current_phase == PHASE.COLLECT:
			# find marker then -> AISLE_DOWN
			pass
		elif cls.current_phase == PHASE.DROPOFF:
			# find exit (2 shelves no marker) then -> AISLE_OUT
			pass
		return cls.current_state, debug_img



	@classmethod
	def AISLE_DOWN_start(cls):
		Specific.leds(0b100)
	
	@classmethod
	def AISLE_DOWN_update(cls, delta, debug_img, visout):
		# move forward avoid obstacles. 
		# try to move towards the marker, if we can't see the marker then use the longest safe path
		# Once we reach target distance (or less) queue a blind_move into collect_item into blind_move to face out
		# For bay3 an addition forward move is required
		dist = None
		if not cls.checkContour(visout.contoursLoadingArea) and visout.marker_bearing is not None:
			fwd, rot = cls.move_into_path(visout.marker_bearing, debug_img, visout.obstacles)
			dist = visout.marker_distance / 100
			
		else:
			fwd, rot = cls.move_into_path(0, debug_img, visout.obstacles)
			dist = 9999
			# print ("This is a problem")
			
		if dist is None:
			print ("This is a problem")


		if cls.target_bay == 3:
			margin = 0.4
			thresh = cls.target_bay_distance/100 + margin # thresh to start blind move
			d = cls.bay_width/2/100

			if dist <= thresh:
				if cls.target_side == "Right":
					PathProcess.new_path([(cls.MAX_ROBOT_VEL, 0, margin-d, None), (cls.MAX_ROBOT_VEL/2, cls.MAX_ROBOT_VEL/0.155, None, pi/2)])
				else:
					PathProcess.new_path([(cls.MAX_ROBOT_VEL, 0, margin-d, None), (cls.MAX_ROBOT_VEL/2, -cls.MAX_ROBOT_VEL/0.155, None, pi/2)])
				cls.next_state = STATE.COLLECT_ITEM
				return STATE.BLIND_MOVE, debug_img
			else:
				cls.set_velocity(fwd, rot, delta, fwdlen = dist-thresh)
		else:
			if visout.marker_bearing is not None and dist <= cls.target_bay_distance/100:
				return STATE.COLLECT_ITEM, debug_img
			else:
				cls.set_velocity(fwd, rot, delta, fwdlen = dist-cls.target_bay_distance/100)
			
		
		return STATE.AISLE_DOWN, debug_img



	@classmethod
	def BLIND_MOVE_start(cls):
		Specific.leds(0b010)
	
	@classmethod
	def BLIND_MOVE_update(cls, delta, debug_img, visout):
		# This one is more of an "action" than a state
		# Use PathProcess and switch to another state when the movement is done

		if PathProcess.completed:
			return cls.next_state, debug_img

		return STATE.BLIND_MOVE, debug_img


	@classmethod
	def AVOID_MOVE_start(cls):
		# our target should be given by a point x,y relative to our current position and heading
		# x is distance in front and y is left-right
		# we set our position in pathprocess to the negative of this and try to navigate towards 0
		# can either terminate when estimated distance to the target is less than a threshold,
		# or when the estimated traversed distance exceeds a threshold.

		# PathProcess.localize(-cls.am_target_x,cls.am_target_y, 0)

		# if not hasattr(cls, 'am_proximity_thresh'):
		# 	cls.am_proximity_thresh = None
		# if not hasattr(cls, 'am_traversed_thresh'):
		# 	cls.am_traversed_thresh = None
		Specific.leds(0b010)
		cls.avoid_moved = 0

		pass
	
	@classmethod
	def AVOID_MOVE_update(cls, delta, debug_img, visout):
		# like blind move but should avoid shelves and obstacles.
		# less strict about the movement obviously. We have a target direction and distance
		# target direction should be tracked through rotations
		# when we have moved target distance or get stuck then go to next state.

		# THis should also serve as our biased_move function to be able to move forward while turning left around the aisle. 

		# x, y, h = PathProcess.get_current_track()
		#cls.forced_avoidance_corner_tracking(np.array([s[0] for s in visout.shelfCorners]))
		

		# if cls.am_traversed_thresh is not None and (x+cls.am_target_x)**2+(y+cls.am_target_y)**2 >= cls.am_traversed_thresh**2:
		# 	return cls.next_state, debug_img
		# elif cls.am_proximity_thresh is not None and x**2+y**2 <= cls.am_proximity_thresh**2:
		# 	return cls.next_state, debug_img
		# else:
		# 	bearing = h - np.arctan2(-y, -x)
		# 	cv2.drawMarker(debug_img, (int(bearing*SCREEN_WIDTH/FOV_HORIZONTAL + SCREEN_WIDTH/2), SCREEN_HEIGHT - int(200*np.linalg.norm(np.array([x,y])))), (255,0,0), cv2.MARKER_DIAMOND, 12)

		if visout.detected_wall is not None:
			fwd, rot = cls.move_into_path((visout.detected_wall[0][0]-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH, debug_img, visout.obstacles)
			# cls.set_velocity(fwd, rot)
		else:
			fwd, rot = cls.move_into_path(0, debug_img, visout.obstacles)
			# cls.set_velocity(fwd, rot)

		#fwd, rot, dist = cls.move_into_path("longest", debug_img, visout.contours, handle_outer=True, handle_outer_value = 1, force_forward=True) # handle outer stuff means empty column (wall) is treated as far away.

		#print(f"{x = :.3f}, \t{y = :.3f}, \t{h = :.3f}, \t{bearing = :.3f}, \t{fwd = :.3f}, \t{rot = :.3f}, \tdist = {dist if dist is not None else -8888:.3f}")

		# Turn bearing then go straight...
		# cls.set_velocity(0,0)
		# PathProcess.new_path([(fwd, rot, None, rot), (fwd, 0, None, None)])
		cls.set_velocity(fwd, rot)

		# fwd, rot, dist = cls.move_into_path('longest', debug_img, visout.contours, handle_outer=False, force_forward=True)
		# cls.set_velocity(fwd,rot)

		cls.avoid_moved += delta * fwd
		if cls.avoid_moved >= cls.avoid_dist:
			return cls.next_state, debug_img

		return STATE.AVOID_MOVE, debug_img
	


	@classmethod
	def COLLECT_ITEM_start(cls):
		Specific.leds(0b010)
		if cls.target_bay != 3:
			cls.collect_item_stage = 0
			if cls.target_side == 'Right':
				cls.set_velocity(0,cls.MAX_ROBOT_ROT,rotlen=2*pi/3)
			else:
				cls.set_velocity(0,-cls.MAX_ROBOT_ROT,rotlen=2*pi/3)
		else:
			cls.set_velocity(0,0)
			cls.collect_item_stage = 1
			cls.face_start = time.time()
	
	@classmethod
	def COLLECT_ITEM_update(cls, delta, debug_img, visout):
		# Attempt to collect the item.

		if debug_img is not None and cls.checkContour(visout.contoursItem):
			cv2.drawContours(debug_img, visout.contoursItem, -1, (255,151,0), 1)

		if cls.collect_item_stage == 0:
			# Turn to bay
			if PathProcess.completed:
				cls.set_velocity(0,0)
				cls.collect_item_stage += 1
				cls.face_start = time.time()
		elif cls.collect_item_stage == 1:
			
			if cls.checkContour(visout.contoursItem):
				
				def item_center_x(cont):
					x, y, w, h = cv2.boundingRect(cont)
					return x + w/2

				largest_item = min(visout.contoursItem, key=lambda cont:abs(item_center_x(cont) - SCREEN_WIDTH/2))

				x, y, w, h = cv2.boundingRect(largest_item)
				cx = x + w/2

				if debug_img is not None:
					cv2.drawMarker(debug_img, (int(cx), int(y+h/2)), (255,151,0), cv2.MARKER_STAR, 12)

				bearing = (cx - SCREEN_WIDTH/2) * FOV_HORIZONTAL/SCREEN_WIDTH
				
				if PathProcess.completed or time.time() - cls.face_start >= 3.0:
					cls.set_velocity(0,0)
					cls.collect_item_stage += 1
				else:
					error = bearing - cls.ITEM_BIAS
					speed = cls.Kp2 * error
					if abs(speed) < cls.MIN_ROTATION:
						speed = math.copysign(cls.MIN_ROTATION, speed)
					cls.set_velocity(0, speed, rotlen=abs(bearing))

			else:
				print("At bay but can't see item")
				if cls.target_side == 'Right':
					cls.set_velocity(0,-cls.MAX_ROBOT_ROT)
				else:
					cls.set_velocity(0,cls.MAX_ROBOT_ROT)

		elif cls.collect_item_stage == 2:
			# Lower item collection
			# Not implemented
			Specific.lifter_set(cls.target_height)
			cls.fwd_start = time.time()
			cls.collect_item_stage += 1
		elif cls.collect_item_stage == 3:
			# Move forward


			if cls.checkContour(visout.contoursItem):
				def item_center_x(cont):
						x, y, w, h = cv2.boundingRect(cont)
						return x + w/2

				largest_item = min(visout.contoursItem, key=lambda cont:abs(item_center_x(cont) - SCREEN_WIDTH/2))

				x, y, w, h = cv2.boundingRect(largest_item)
				cx = x + w/2

				if debug_img is not None:
					cv2.drawMarker(debug_img, (int(cx), int(y+h/2)), (255,151,0), cv2.MARKER_STAR, 12)

				bearing = (cx - SCREEN_WIDTH/2) * FOV_HORIZONTAL/SCREEN_WIDTH
			else:
				bearing = 0
			
			bearing = bearing - cls.ITEM_BIAS

			if time.time() - cls.fwd_start >= 3.0:
				cls.set_velocity(0,0)
				cls.collect_item_stage += 1
			else:
				fwd, rot = cls.move_into_path(bearing, debug_img, visout.obstacles)
				cls.set_velocity(fwd, rot)
				
		elif cls.collect_item_stage == 4:
			# Close gripper
			Specific.gripper_close(2.5)

			if cls.target_bay != 3:
				cls.set_velocity(-2*cls.MAX_ROBOT_VEL/3,0,fwdlen=0.20)
			else:
				if cls.target_side == 'Right':
					PathProcess.new_path([(-cls.MAX_ROBOT_VEL/2, 0, 0.05, None), (-cls.MAX_ROBOT_VEL/2, -cls.MAX_ROBOT_VEL/0.155, None, pi/2)])
				else:
					PathProcess.new_path([(-cls.MAX_ROBOT_VEL/2, 0, 0.05, None), (-cls.MAX_ROBOT_VEL/2, cls.MAX_ROBOT_VEL/0.155, None, pi/2)])
			cls.collect_item_stage += 1
		elif cls.collect_item_stage == 5:
			# Move backwards
			if PathProcess.completed:
				if cls.target_bay != 3:
					if cls.target_side == 'Right':
						cls.set_velocity(0,cls.MAX_ROBOT_ROT, rotlen=pi/2)
					else:
						cls.set_velocity(0,-cls.MAX_ROBOT_ROT, rotlen=pi/2)
				else:
					if cls.target_side == 'Right':
						cls.set_velocity(0,cls.MAX_ROBOT_ROT, rotlen=pi)
					else:
						cls.set_velocity(0,-cls.MAX_ROBOT_ROT, rotlen=pi)

				cls.collect_item_stage += 1
		elif cls.collect_item_stage == 6:
			# Move backwards
			if PathProcess.completed:
				Specific.lifter_set(2)

				if cls.target_side == 'Right':
					cls.set_velocity(0,cls.MAX_ROBOT_ROT)
				else:
					cls.set_velocity(0,-cls.MAX_ROBOT_ROT)
				
				cls.collect_item_stage += 1
		elif cls.collect_item_stage == 7:
			# Face outwards
			if visout.marker_bearing is None and visout.detected_wall is not None:
				cls.current_phase = PHASE.DROPOFF
				return STATE.AISLE_OUT, debug_img

		return STATE.COLLECT_ITEM, debug_img



	@classmethod
	def AISLE_OUT_start(cls):
		Specific.leds(0b001)
		cls.last_shelf_side = None
		cls.aisle_out_stage = -1
	
	@classmethod
	def AISLE_OUT_update(cls, delta, debug_img, visout):
		# We should be able to see two shelves and no marker. (or marker and packing station)
		# Move towards the exit point, which is the center of the gap between the shelves
		# When we can no longer see the shelves, queue a blind move forwards into -> LOST_OUTSIDE_AISLE
		
		if cls.aisle_out_stage == -1:
			if visout.detected_shelves is None:
				cls.next_state = STATE.APPROACH_PACKING
				cls.avoid_dist = 0.20
				return STATE.AVOID_MOVE, debug_img
			cls.aisle_out_stage = 0

		if cls.aisle_out_stage == 0:

			if visout.detected_wall is not None:
				fwd, rot = cls.move_into_path((visout.detected_wall[0][0]-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH, debug_img, visout.obstacles)
				cls.set_velocity(fwd, rot)
			else:
				fwd, rot = cls.move_into_path(0, debug_img, visout.obstacles)
				cls.set_velocity(fwd, rot)

			if visout.detected_shelves is None:
				cls.next_state = STATE.APPROACH_PACKING
				cls.avoid_dist = 0.20 + 0.03 * cls.target_aisle
				return STATE.AVOID_MOVE, debug_img



		return STATE.AISLE_OUT, debug_img



	@classmethod
	def APPROACH_PACKING_start(cls):
		Specific.leds(0b001)
		cls.loading_area_approach_stage = 0
		cls.set_velocity(0, cls.MAX_ROBOT_ROT, rotlen = 2*pi)
	
	@classmethod
	def APPROACH_PACKING_update(cls, delta, debug_img, visout):
		# We should be able to see the packing station at all times.
		# Move towards the center of the packing station until its bounding box touches left and right of screen
		# then move towards the marker until the packing station's bounding box touches the bottom of the screen
		# If collect then -> DROP_ITEM otherwise LOST_OUTSIDE_AISLE

		if cls.loading_area_approach_stage == 0:
			# Find loading area
			if cls.checkContour(visout.contoursLoadingArea):
				cls.loading_area_approach_stage += 1
			elif PathProcess.completed:
				return STATE.LOST_OUTSIDE_AISLE, debug_img
		elif cls.loading_area_approach_stage == 1:
			# move towards it
			if not cls.checkContour(visout.contoursLoadingArea):
				cls.loading_area_approach_stage -= 1
				cls.set_velocity(0, cls.MAX_ROBOT_ROT, rotlen = 2*pi)
			else:
				x,y,w,h = cv2.boundingRect(visout.contoursLoadingArea[0])
				if y+h >= SCREEN_HEIGHT-1:
					cls.loading_area_approach_stage += 1
					cls.face_start = time.time()
					cls.set_velocity(0,0)
				else:
					cx = x + w/2
					bearing = (cx - SCREEN_WIDTH/2) * FOV_HORIZONTAL/SCREEN_WIDTH
					fwd, rot = cls.move_into_path(bearing, debug_img, visout.obstacles)
					cls.set_velocity(fwd, rot, delta)
		elif cls.loading_area_approach_stage == 2:
			# face marker

			if visout.marker_bearing is not None:
				
				if PathProcess.completed or time.time() - cls.face_start >= 3.0:
					cls.set_velocity(0,0)
					return STATE.DROP_ITEM, debug_img
				else:
					error = visout.marker_bearing - cls.ITEM_BIAS
					speed = cls.Kp2 * error
					if abs(speed) < cls.MIN_ROTATION:
						speed = math.copysign(cls.MIN_ROTATION, speed)
					cls.set_velocity(0, speed, rotlen=abs(visout.marker_bearing))
			else:
				print("At packing but can't see marker")
				cls.set_velocity(0, cls.MAX_ROBOT_ROT)

		return STATE.APPROACH_PACKING, debug_img


	completed_items = 0

	@classmethod
	def DROP_ITEM_start(cls):
		Specific.leds(0b001)
		cls.drop_item_stage = 0
		cls.drop_start = time.time()
		cls.set_velocity(1.1*cls.MAX_ROBOT_VEL,0)#,fwdlen=0.60) go until we can't see marker
	
	@classmethod
	def DROP_ITEM_update(cls, delta, debug_img, visout):
		# Forward, drop, backwards, turn, forwards, -> LOST_OUTSIDE_AISLE

		desired_marker_dist = 0.25 # m from packing marker (not real distance but judged distance)

		if cls.drop_item_stage == 0:
			# Move forward
			if PathProcess.completed or (visout.marker_bearing is not None and visout.marker_distance/100 <= desired_marker_dist) or time.time() - cls.drop_start >= 8.0:
				cls.set_velocity(0,0)
				cls.drop_item_stage = 2
			else:
				if visout.marker_bearing is not None:
					fwd, rot = cls.move_into_path(visout.marker_bearing, debug_img, visout.obstacles)
					dist = visout.marker_distance/100 - desired_marker_dist
					cls.set_velocity(fwd*1.4, rot*1.4, fwdlen=dist)
				else:
					pass
				
				
		# elif cls.drop_item_stage == 1:
		# 	if PathProcess.completed:
		# 		cls.set_velocity(0,0)
		# 		cls.drop_item_stage += 1
		elif cls.drop_item_stage == 2:
			# Drop item
			Specific.gripper_open(0.25)
			time.sleep(0.5)
			Specific.gripper_close()
			Specific.gripper_open()
			# Move back
			if cls.target_aisle == 1: # This is the aisle we came from
				cls.set_velocity(-cls.MAX_ROBOT_VEL,0,fwdlen=0.43)
			elif cls.target_aisle == 2:
				cls.set_velocity(-cls.MAX_ROBOT_VEL,0,fwdlen=0.50)
			elif cls.target_aisle == 3:
				cls.set_velocity(-cls.MAX_ROBOT_VEL,0,fwdlen=0.55)
			
			cls.drop_item_stage = 3
		# elif cls.drop_item_stage == 2:
		# 	# Move backward
		# 	if PathProcess.completed:
		# 		if cls.target_aisle == 1: # This is the aisle we came from
		# 			PathProcess.new_path([(0, -cls.MAX_ROBOT_ROT, None, 2*pi/5)])
		# 		elif cls.target_aisle == 2:
		# 			PathProcess.new_path([(0, -cls.MAX_ROBOT_ROT, None, 2*pi/3)])
		# 		elif cls.target_aisle == 3:
		# 			PathProcess.new_path([(0, -cls.MAX_ROBOT_ROT, None, 5*pi/6)])
		# 		cls.drop_item_stage += 1
		elif cls.drop_item_stage == 3:
			# Turn around
			if PathProcess.completed:
				if cls.completed_items >= 3:
					cls.set_velocity(0,0)
					time.sleep(0.5)
					Specific.play_song()
					time.sleep(0.5)

					return STATE.VEGETABLE, debug_img
				else:
					cls.completed_items += 1
					cls.current_phase = PHASE.COLLECT
					cls.current_instruction = (cls.current_instruction % (len(cls.instructions)-1)) + 1
					cls.process_instruction()

					# cls.avoid_dist = 0.4
					# cls.next_state = STATE.LOST
					# return STATE.AVOID_MOVE, debug_img
					return STATE.LOST, debug_img

		return STATE.DROP_ITEM, debug_img


	#endregion