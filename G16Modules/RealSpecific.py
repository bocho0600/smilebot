import picamera2
import cv2
from .Globals import *
import time
from threading import Thread
import numpy as np
from .RP2040 import I2C
from .Mobility import MobilityModule


I2C.init()
from .ItemCollection import ItemCollectionModule


class RealSpecific:

	# Specific should implement:
	# get_frame
	# set_velocity
	# start
	# update
	# stop

	camera = None
	frameGrabber = None
	

	@classmethod
	def get_image(cls):
		return cls.frameGrabber.getCurrentFrame()
	
	@classmethod
	def get_new_image(cls):
		while not cls.frameGrabber.newFrameAvailable:
			time.sleep(0.01)
		return cls.frameGrabber.getCurrentFrame()

	@classmethod
	def initialize_camera(cls, frame_height=820, frame_width=616, format='XRGB8888'):
		# Create a camera object and store it as an instance variable
		cls.camera = picamera2.Picamera2()
		config = cls.camera.create_video_configuration(main={"format": format, "size": (frame_height, frame_width)})
		cls.camera.configure(config)
		
		
		# cls.cap.set_controls({"ExposureTime": 11000, "AnalogueGain": 1.5,  "ColourGains": (1.22,2.12)})
		# cls.cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})
		cls.camera.set_controls({"ExposureTime": 70000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})

		cls.image_width = frame_width
		cls.image_center = cls.image_width // 2 # Calculate the center of the image
		cls.camera.start()
	
	@classmethod
	def set_velocity(cls, fwd, rot):
		a = 470 # multiply m/s to get PWM value 0 to 255
		rot_ms = rot * 0.155/2 # rad/s to m/s
		MobilityModule.Move(int(fwd*a), int(rot_ms*a))
		pass

	@classmethod
	def start(cls, grabber = True):

		cls.initialize_camera()
		if grabber:
			cls.frameGrabber = CamFrameGrabber(cls.camera, SCREEN_WIDTH, SCREEN_HEIGHT)
			cls.frameGrabber.start()
		
		
		time.sleep(0.3)
		cls.leds(0b101)
		time.sleep(0.3)
		cls.leds(0b010)
		time.sleep(0.3)
		cls.leds(0b101)

		ItemCollectionModule.init()
		cls.leds(0b000)
		time.sleep(0.3)




	@classmethod
	def update(cls):
		pass
		# if ItemCollectionModule.is_initialized:
		# 	ItemCollectionModule.update()

	@classmethod
	def end(cls):
		cls.frameGrabber.stop()
		cls.camera.close()
		cls.set_velocity(0,0)
		cls.leds(0b000)



	last_mask = 0b000
	@classmethod
	def leds(cls, mask):
		changed = mask ^ cls.last_mask
		for i in range(3):
			if changed >> i & 1:
				if mask >> i & 1:
					I2C.LedWrite(i+1,"ON")
				else:
					I2C.LedWrite(i+1,"OFF")
		cls.last_mask = mask

	@classmethod
	def lifter_set(cls, h):
		ItemCollectionModule.lifter_set(h)

	@classmethod
	def gripper_stop(cls):
		ItemCollectionModule.gripper_stop()
	
	@classmethod
	def gripper_close(cls, seconds=2.0):
		ItemCollectionModule.gripper_close(seconds)
	
	@classmethod
	def gripper_open(cls, seconds=1.7):
		ItemCollectionModule.gripper_open(seconds)

	@classmethod
	def play_song(cls):
		I2C.PlaySong('1')



# Define the CamFrameGrabber class
class CamFrameGrabber:
	# FOV = number of degrees for camera view
	def __init__(self, camera, width, height):
		# Initialize the camera
		self.camera = camera
		self.cameraStopped = False
		self.gotFrame = False
		self.currentFrame = np.zeros((height, width, 3), np.uint8)
		self.frameid = 0  # Initialize frame ID
		self.newFrameAvailable = False
		self.time_of_frame = 0
		self.fps = 0

		# Capture the first frame
		self.currentFrame = self.camera.capture_array()

	def start(self):
		# Start the image capturing thread
		self.previous_frame_id = -1
		self.grabberThread = Thread(target=self.captureImage, args=()).start()  # Running the camera capturing in background threads
		return self

	def captureImage(self):
		# Continuously capture frames until stopped
		while True:
			if self.cameraStopped:
				return
			
			new_frame = self.camera.capture_array()#RealSpecific.CaptureFrame()
			self.currentFrame = new_frame
			
			self.frameid += 1  # Increment the frame ID after capturing each frame
			self.newFrameAvailable = True
			

	def getCurrentFrame(self):
		# Return the current frame
		self.newFrameAvailable = False
		
		# return self.currentFrame.copy()
		imgFlip = cv2.resize(self.currentFrame, (410, 308))
		imgRGB = imgFlip #cv2.rotate(imgFlip, cv2.ROTATE_180)
		imgHSV = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2HSV)  # Convert to HSV
		RobotView = imgRGB.copy()  # Preserve the original image
		return imgRGB, imgHSV, RobotView

	def getFrameID(self):
		# Return the current frame ID
		return self.frameid

	def stop(self):
		# Stop the camera capture
		self.cameraStopped = True
		if self.grabberThread is not None and self.grabberThread.is_alive():
			self.grabberThread.join()


	def __del__(self):
		# Release the camera and clean up OpenCV windows
		self.camera.stop()
		cv2.destroyAllWindows()
