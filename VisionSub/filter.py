import cv2
import numpy as np
import picamera2

def nothing(x):
    pass

# Initialize PiCamera
def initialize_camera(frame_height=616, frame_width=820, format='XRGB8888'):
    cap = picamera2.Picamera2()
    config = cap.create_video_configuration(main={"format": format, "size": (frame_width, frame_height)})
    cap.configure(config)
    #cap.set_controls({"ExposureTime": 29999, "AnalogueGain": 3.76, "ColourGains": (1.76,1.4)})
    #cap.set_controls({"ExposureTime": 11000, "AnalogueGain": 1.5,  "ColourGains": (1.22,2.12)})
    cap.set_controls({"ExposureTime": 70000, "AnalogueGain": 1,  "ColourGains": (1.4,1.5)}) 
    cap.start()
    return cap

# Create a window
cv2.namedWindow('image', cv2.WINDOW_NORMAL)  # 'WINDOW_NORMAL' allows resizing

# Set the desired size for the window (e.g., 800x600)
cv2.resizeWindow('image', 800, 600)
cv2.namedWindow('Mask')
# Create trackbars for HSV color filtering
cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

# Set default values for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

# Initialize the camera
cap = initialize_camera()

try:
    while True:
        # Capture frame-by-frame
        frame = cap.capture_array()
        frame = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically

        # Get current camera settings (metadata)
        metadata = cap.capture_metadata()
        exposure_time = metadata.get("ExposureTime", "N/A")
        awb_mode = metadata.get("AwbMode", "N/A")
        awb_gains = metadata.get("AwbGains", "N/A")
        
        # Get additional camera settings if available
        analogue_gain = metadata.get("AnalogueGain", "N/A")
        colour_gains = metadata.get("ColourGains", "N/A")

        # Print the current camera settings
        print(f"Current Exposure Time: {exposure_time}")
        print(f"Current AWB Mode: {awb_mode}")
        print(f"Current AWB Gains: {awb_gains}")
        print(f"Current Analogue Gain: {analogue_gain}")
        print(f"Current Colour Gains: {colour_gains}")

        # Get current positions of HSV trackbars
        hMin = cv2.getTrackbarPos('HMin', 'image')
        sMin = cv2.getTrackbarPos('SMin', 'image')
        vMin = cv2.getTrackbarPos('VMin', 'image')
        hMax = cv2.getTrackbarPos('HMax', 'image')
        sMax = cv2.getTrackbarPos('SMax', 'image')
        vMax = cv2.getTrackbarPos('VMax', 'image')

        # Set minimum and maximum HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Convert to HSV format and color threshold
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Print if there is a change in HSV value
        if((phMin != hMin) or (psMin != sMin) or (pvMin != vMin) or (phMax != hMax) or (psMax != sMax) or (pvMax != vMax)):
            print(f"(hMin = {hMin}, sMin = {sMin}, vMin = {vMin}), (hMax = {hMax}, sMax = {sMax}, vMax = {vMax})")
            phMin = hMin
            psMin = sMin
            pvMin = vMin
            phMax = hMax
            psMax = sMax
            pvMax = vMax

        # Display result image
        cv2.imshow('image', result)
        cv2.imshow('Mask', mask)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
finally:
    # Release resources
    cap.close()
    cv2.destroyAllWindows()