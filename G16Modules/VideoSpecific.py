from .Globals import *
import cv2
import time
class VideoSpecific:

    # Specific should implement:
	# get_frame
	# set_velocity
	# start
	# update
	# stop

    video_path = 'VisionSub/videos/rec4.h264'

    @classmethod
    def get_image(cls):
        ret, img = cls.cap.read()
        if not ret:
            cls.cap.release()
            cls.cap = cv2.VideoCapture(cls.video_path)
            ret, img = cls.cap.read()
    
        img = cv2.rotate(img, cv2.ROTATE_180)
        img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT), cv2.INTER_NEAREST)
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        robotview = img #.copy()

        return img, imgHSV, robotview

    
    @classmethod
    def set_velocity(cls, fwd, rot):
        pass
    
    @classmethod
    def start(cls):
        cls.cap = cv2.VideoCapture(cls.video_path)

    @classmethod
    def update(cls):
        time.sleep(0.02)

    @classmethod
    def end(cls):
        cls.cap.release()

    @classmethod
    def leds(cls, mask):
        pass

    @classmethod
    def lifter_set(cls, h):
        pass

