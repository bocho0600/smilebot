from math import pi


SCREEN_WIDTH = 410
SCREEN_HEIGHT = 308

is_simulator = False	
is_hitl = False
is_video = False
is_combined = False

if is_combined:
	CAMERA_FOV = 60
	FOV_HORIZONTAL = CAMERA_FOV * pi/180 # radians

	DIST_X = 0.07 # cameraDistanceFromRobotCenter
	DIST_Z = 0.0752 # cameraHeightFromFloor
	TILT = 1.0 * 3.1415926535 / 180

	from .CombinedSpecific import CombinedSpecific as Specific
elif is_simulator:
	CAMERA_FOV = 60
	FOV_HORIZONTAL = CAMERA_FOV * pi/180 # radians

	DIST_X = 0.07 # cameraDistanceFromRobotCenter
	DIST_Z = 0.0752 # cameraHeightFromFloor
	TILT = 1.0 * 3.1415926535 / 180

	from .SimSpecific import SimSpecific as Specific
elif is_video:
	CAMERA_FOV = 64
	FOV_HORIZONTAL = CAMERA_FOV * pi/180 # radians

	DIST_X = 0.09 # cameraDistanceFromRobotCenter
	DIST_Z = 0.081 # cameraHeightFromFloor
	TILT = -3.785 * pi / 180
	
	from .VideoSpecific import VideoSpecific as Specific
else:
	CAMERA_FOV = 64
	FOV_HORIZONTAL = CAMERA_FOV * pi/180 # radians


	# Brown chassis
	# DIST_X = 0.15 # cameraDistanceFromRobotCenter
	# DIST_Z =  0.089# cameraHeightFromFloor
	# TILT = 1.45 * pi / 180

	# White chassis
	DIST_X = 0.0 # cameraDistanceFromRobotCenter
	DIST_Z =  0.162# cameraHeightFromFloor
	TILT = 3.8 * pi / 180
	
	from .RealSpecific import RealSpecific as Specific

