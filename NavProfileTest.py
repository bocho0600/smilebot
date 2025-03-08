import cv2
import numpy as np
import time
from G16Modules.Globals import *
from G16Modules.Vision import VisionModule
from G16Modules.Navigation import NavigationModule, STATE
#import RP2040 as I2C
import VisionSub.csvread as csvread
import cProfile
import re

dist_map = None
safety_map = None

def profiled():
	global safety_map
	
	for i in range(100):
		safety_map = NavigationModule.expand_safety(dist_map)

if __name__ == "__main__": # Run the main function
	CSV = csvread.CSVReader('Order_1.csv')
	CSV.read_csv()
	instructions = CSV.RobotInstruction() # Generating robot instructions and print instructions

	#i2c = I2C.I2C()

	# Possibly override color ranges for simulator
	if hasattr(Specific, 'color_ranges'):
		VisionModule.color_ranges = Specific.color_ranges


	try:
		
		Specific.start()
		instruction = instructions[2] # 1 2 4
		
		time.sleep(1)

		NavigationModule.init(STATE.WANDER, instruction)

		
		robotview, visout = VisionModule.Pipeline(False)

		cv2.imshow("View", robotview)
		cv2.waitKey(1)

		points = VisionModule.combine_contour_points(visout.contours, exclude_horizontal_overlap=False)
		points = VisionModule.handle_outer_contour(points)
		points, projected_floor = VisionModule.project_and_filter_contour(points)
		if points is not None and points.shape[0] > 3:
			dist_map = VisionModule.get_dist_map(points, projected_floor) # dist map column 0 is dist, column 1 is real point
			cProfile.run('profiled()')
			# profiled()
		else:
			print("No shelf points found")
		
		robotview = cv2.polylines(robotview, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map[:,0]/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
		robotview = cv2.polylines(robotview, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw
		
		cv2.imshow("View", robotview)
		cv2.waitKey(0)

		Specific.update()


	finally:
		Specific.end()
