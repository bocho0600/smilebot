import time
import picamera2
import cv2
import numpy as np

def onClick(event, x, y, flags, params):
	global image_points
	if event == cv2.EVENT_LBUTTONDOWN:
		print(x,y)
		if not found_homography:
			image_points.append([x,y])
		else:
			print('Predicted point')
			pred_point = cv2.perspectiveTransform(np.float32([[x,y]]).reshape(-1,1,2), M)
			print(pred_point)

# define position of points in the ground plane relative to the robot
ground_points = np.float32([[-18,47],[18,47],[-18,150],[18,150]]) # cm
# click the points in the image (order is important) to populate the image points
image_points = []

# intialise the camera
cap = picamera2.Picamera2()

# add camera configuration settings here
config = cap.create_video_configuration(main={"format":'XRGB8888', "size":(820,616)})
cap.configure(config)
cap.start()

# capture
frame = cap.capture_array()

# reduce size to speed up	
#frame = cv2.rotate(frame, cv2.ROTATE_180)

# display
cv2.imshow("Image", frame)
cv2.setMouseCallback("Image", onClick)

# click four points and then fit the homography
found_homography = False
while True:
	cv2.waitKey(1)
	if len(image_points) == 4:
		if not found_homography:
			M, mask = cv2.findHomography(np.float32(image_points).reshape(-1,1,2), ground_points.reshape(-1,1,2))
			found_homography = True
			print("found homography")
			print(M)
			
# shutdown
cap.close()
cv2.destroyAllWindows()
