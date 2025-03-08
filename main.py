import cv2
import numpy as np
import time
from G16Modules.Globals import *
from G16Modules.Vision import VisionModule
from G16Modules.Navigation import NavigationModule, STATE
import VisionSub.csvread as csvread
from G16Modules.PathProcess import PathProcess
import math

def main(): # Main function

	# Possibly override color ranges for simulator
	if hasattr(Specific, 'color_ranges'):
		VisionModule.color_ranges = Specific.color_ranges


	try:
		
		Specific.start()
		
		VisionModule.calculate_projection_transform()
		starting_instruction = 4

		
		pipeline = 'nav'
		draw = True

		
		if pipeline == 'debug_distmap':
			NavigationModule.init(STATE.VEGETABLE, instructions, starting_instruction)
		elif pipeline == 'debug':
			NavigationModule.init(STATE.VEGETABLE, instructions, starting_instruction)
		elif pipeline == 'nav':
			NavigationModule.init(STATE.LOST, instructions, starting_instruction)
		elif pipeline == "pcontrol":
			NavigationModule.init(STATE.VEGETABLE, instructions, starting_instruction)
		t1 = time.time()

		while True:
		
			if pipeline == 'debug_distmap':
				# Run vision and most CPU-intensive nav code but don't move
				robotview, visout = VisionModule.Pipeline(draw)
				if robotview is None:
					continue

				points = VisionModule.combine_contour_points(visout.contoursShelf, exclude_horizontal_overlap=False)
				#points = VisionModule.handle_outer_contour(points)
				points, projected_floor = VisionModule.project_and_filter_contour(points)
				if points is not None and points.shape[0] > 3:
					dist_map = VisionModule.get_dist_map(points, projected_floor) # dist map column 0 is dist, column 1 is real point
					#safety_map = NavigationModule.expand_safety(dist_map)

					if draw:
						robotview = cv2.polylines(robotview, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map[:,0]/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
						#robotview = cv2.polylines(robotview, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw
						projection_image = np.zeros(robotview.shape)

						cv2.line(projection_image, (0, SCREEN_HEIGHT-100), (SCREEN_WIDTH-1, SCREEN_HEIGHT-100), (0,0,255), 1)		
						for i in range(len(projected_floor)):
							cv2.drawMarker(projection_image, tuple((np.array([SCREEN_WIDTH/2, SCREEN_HEIGHT])+np.array([100, -100])*projected_floor[i, ::-1]).astype(np.int32)), (255,255,255), cv2.MARKER_DIAMOND, 4)
						# for point in VisionModule.project_to_ground(np.array([s[0] for s in visout.shelfCorners])):
						# 	cv2.drawMarker(projection_image, tuple((np.array([SCREEN_WIDTH/2, SCREEN_HEIGHT])+np.array([100, -100])*point[::-1]).astype(np.int32)), (0, 0,255), cv2.MARKER_CROSS, 7)

				else:
					print("No shelf points found")
					if draw:
						projection_image = np.zeros(robotview.shape)
				
				
				if draw:
					VisionModule.ExportImage("RobotView", robotview, FPS = True)
					VisionModule.ExportImage("Map", projection_image, FPS = False)
				else:
					t2 = time.time()
					print(f"FPS: {1.0/(t2-t1):.1f}")
					t1 = t2
			elif pipeline == 'debug':
				# Run vision and most CPU-intensive nav code but don't move
				NavigationModule.current_state = STATE.VEGETABLE
				NavigationModule.set_velocity(0,0)
				robotview, img2= VisionModule.DebugPipeline(draw)
				if robotview is None:
					continue
				

				if draw:
					VisionModule.ExportImage("RobotView", robotview, FPS = True)
					VisionModule.ExportImage("Wallmask", img2, FPS = False)
				else:
					t2 = time.time()
					print(f"FPS: {1.0/(t2-t1):.1f}")
					t1 = t2
			elif pipeline == 'nav':
				# Full navigation move to the desired shelf
				robotview, visout = VisionModule.Pipeline(draw)
				if robotview is None:
					continue
				img2 = visout.WallMask
				
				# print(marker_distance, marker_bearing)
				
				if draw:
					robotview, img2 = NavigationModule.update(robotview, visout)
					VisionModule.ExportImage("RobotView", robotview, FPS = True)
					if img2 is not None:
						VisionModule.ExportImage("Img 2", img2, FPS = False)
				else:
					robotview = NavigationModule.update(None, visout)

					t2 = time.time()
					print(f"FPS: {1.0/(t2-t1):.1f}")
					t1 = t2
			elif pipeline == "pcontrol":
				robotview, visout = VisionModule.Pipeline(draw)
				if robotview is None:
					continue

				def item_center_x(cont):
					x, y, w, h = cv2.boundingRect(cont)
					return x + w/2

				largest_item = min(visout.contoursItem, key=lambda cont:abs(item_center_x(cont) - SCREEN_WIDTH/2))

				x, y, w, h = cv2.boundingRect(largest_item)
				cx = x + w/2

				if draw:
					cv2.drawMarker(robotview, (int(cx), int(y+h/2)), (255,151,0), cv2.MARKER_STAR, 12)

				bearing = (cx - SCREEN_WIDTH/2) * FOV_HORIZONTAL/SCREEN_WIDTH

				if bearing is not None:
				
					if PathProcess.completed:
						NavigationModule.set_velocity(0,0)
						
					else:
						speed = NavigationModule.Kp2 * bearing
						if abs(speed) < NavigationModule.MIN_ROTATION:
							speed = math.copysign(NavigationModule.MIN_ROTATION, speed)
						NavigationModule.set_velocity(0, speed, rotlen=abs(bearing))
				else:
					NavigationModule.set_velocity(0, 0)



				if draw:
					VisionModule.ExportImage("RobotView", robotview, FPS = True)
				else:
					t2 = time.time()
					print(f"FPS: {1.0/(t2-t1):.1f}")
					t1 = t2


			Specific.update()

			

			if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
				break

	finally:
		NavigationModule.end()
		Specific.end()

if __name__ == "__main__": # Run the main function
	CSV = csvread.CSVReader('Order_2.csv')
	CSV.read_csv()
	instructions = CSV.RobotInstruction() # Generating robot instructions and print instructions

	main()
