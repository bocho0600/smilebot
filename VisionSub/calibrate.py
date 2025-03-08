import numpy as np
import cv2
import time
from picamera2 import Picamera2

# Create a Picamera2 object
cap = Picamera2()

# Configure the camera
frameWidth, frameHeight = 320 * 2, 240 * 2
config = cap.create_video_configuration(main={"format": 'XRGB8888', "size": (frameWidth, frameHeight)})
cap.configure(config)

# Start the camera
cap.start()

captured_image = None  # Variable to store the captured image

valueHSV = set()  # This is a set of HSV values captured by the user
colorHSV = (np.array([0, 0, 0]), np.array([0, 0, 0]))  # This is the HSV range of the color
map = ((np.array([0, 0, 0]), np.array([0, 0, 0])), (np.array([0, 0, 0]), np.array([0, 0, 0])))

color_ranges = {
    'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
    'yellow': (np.array([20, 120, 153]), np.array([25, 233, 218])),
    'blue': (np.array([90, 136, 9]), np.array([120, 255, 94])),
    'green': (np.array([40, 64, 11]), np.array([70, 255, 99])),
    'orange1': (np.array([0, 158, 45]), np.array([13, 255, 235])),
    'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
    'black': (np.array([0, 0, 43]), np.array([179, 55, 109]))
}

def add_point_in(point):
    global valueHSV
    valueHSV.add((point[0], point[1], point[2]))
    print("Point added in: ", valueHSV)

def OnMouseClick(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        paramHSV = cv2.cvtColor(param, cv2.COLOR_BGR2HSV)
        add_point_in(paramHSV[y, x])
        update_thresh()

def update_thresh():
    global colorHSV, map
    if len(valueHSV) > 0:
        arr = np.array(list(valueHSV))
        colorHSV = (arr.min(0), arr.max(0))  # Get the min and max values of the HSV points
        print("Color HSV: ", colorHSV)
        map = colorHSV
        if captured_image is not None:
            captured_imageHSV = cv2.cvtColor(captured_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(captured_imageHSV, colorHSV[0], colorHSV[1])
            cv2.imshow("Mask", cv2.bitwise_and(captured_image, captured_image, mask=mask))
    else:
        colorHSV = (np.array([0, 0, 0]), np.array([0, 0, 0]))
        print("Color HSV: ", colorHSV)

    return colorHSV

def reset():
    global valueHSV, colorHSV, map
    valueHSV = set()
    colorHSV = (np.array([0, 0, 0]), np.array([0, 0, 0]))
    map = colorHSV
    print("Resetting points")
    if captured_image is not None:
        captured_imageHSV = cv2.cvtColor(captured_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(captured_imageHSV, colorHSV[0], colorHSV[1])
        cv2.imshow("Mask", cv2.bitwise_and(captured_image, captured_image, mask=mask))

if __name__ == "__main__":
    while True:
        t1 = time.time()  # for measuring fps

        frame = cap.capture_array()  # Capture a single image frame
        
        # Display the obtained frame in a window called "CameraImage"
        cv2.imshow("Image", frame)
        cv2.waitKey(1)  # Make the program wait for 1ms before continuing (also required to display image).

        fps = 1.0 / (time.time() - t1)  # Calculate frame rate
        print("Frame Rate: ", int(fps), end="\r")

        key = cv2.waitKey(1) & 0xFF  # Wait for a key press

        if key == ord('c'):  # Capture the image if 'c' is pressed
            captured_image = frame.copy()
            print("Image captured!")
            cv2.destroyWindow("Image")
            break

    # If an image was captured, display it in a new window
    if captured_image is not None:
        cv2.imshow("Captured Image", captured_image)
        cv2.setMouseCallback("Captured Image", OnMouseClick, captured_image)
        cv2.waitKey(0)  # Wait until a key is pressed to close the window

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Exit the loop if 'q' is pressed
            break
        elif key == ord('s'):  # Save the HSV range if 's' is pressed
            print("Color HSV: ", colorHSV)
            print("Color Threshold stored in calibrate.csv")
            # Flatten the arrays for saving to CSV
            colorHSV_flattened = np.hstack(colorHSV).reshape(1, -1)
            np.savetxt("calibrate.csv", colorHSV_flattened, delimiter=",", fmt='%d')
        elif key == ord('r'):  # Reset the HSV range if 'r' is pressed
            reset()
            update_thresh()
            print("Resetting color HSV to: ", colorHSV)

    cap.close()
    cv2.destroyAllWindows()
