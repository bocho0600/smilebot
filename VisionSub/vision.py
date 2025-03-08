import cv2
from math import cos,sin,pi,atan2,degrees
import numpy as np
import time
from threading import Thread
import picamera2

#from threading import Thread
# MADE BY KELVIN LE, QUT EGB320 GROUP16 
# StudentID: n11429984
# Email: minhnguyen.le@qut.edu.au

class VisionModule:
    ground_points = np.float32([[-18,47],[18,47],[-18,150],[18,150]]) #off-set in cm
    homography_matrix = np.array([[-2.11252555e-02,  6.33812791e-03,  7.01814123e+00],[ 1.80590083e-04, -8.03575074e-03, -1.43661305e+01]
    ,[ 2.56484473e-05, -4.57580665e-03,  1.00000000e+00]])
    color_ranges = {
        #'wall': (np.array([34, 0, 211]), np.array([45, 74, 255])),
        'wall': (np.array([33, 0, 0]), np.array([41, 62, 255])),
        'floor': (np.array([0, 0, 0]), np.array([179, 255, 255])),
        'yellow': (np.array([25, 108, 224]), np.array([33, 255, 255])),
        'blue': (np.array([81, 0, 0]), np.array([116, 255, 255])),
        'green': (np.array([55, 79, 0]), np.array([70, 255, 255])),
        'orange1': (np.array([0, 100, 0]), np.array([20, 255, 255])),
        'orange2': (np.array([165, 100, 0]), np.array([180, 255, 255])),
        'black': (np.array([38,31, 45]), np.array([66, 121 , 88]))
    }


    focal_length = 30 #cm
    real_circle_diameter = 70 #cm

    def __init__(self):
        self.cap = None  # Initialize the camera object as an instance variable
        self.t1 = None

    def draw_crosshair(self, frame, color=(255, 255, 255), thickness=2):
        # Get the dimensions of the frame
        height, width = frame.shape[:2]
        
        # Calculate the center of the frame
        center_x = width // 2
        center_y = height // 2
        FrameCenter = (center_x, center_y)
        # Define the length of the crosshair arms
        crosshair_length = 5
        
        # Draw the vertical line of the crosshair

        cv2.drawMarker(frame, FrameCenter, color, markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
        # Draw the horizontal line of the crosshair
        return FrameCenter


    def CaptureImage(self):
        self.frame = self.cap.capture_array()  # Capture an image from the camera
        self.frame = cv2.resize(self.frame, (410, 308))
        #self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
        return self.frame


    def initialize_camera(self, frame_width=820, frame_height=616, format='XRGB8888'):
        # Create a camera object and store it as an instance variable
        self.cap = picamera2.Picamera2()
        config = self.cap.create_video_configuration(main={"format": format, "size": (frame_width, frame_height)})
        self.cap.configure(config)
   
        
        #self.cap.set_controls({"ExposureTime": 11000, "AnalogueGain": 1.5,  "ColourGains": (1.22,2.12)})
        self.cap.set_controls({"ExposureTime": 70000, "AnalogueGain": 1,  "ColourGains": (1.4,1.5)}) 
        #self.cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})
        #self.cap.set_controls({"ExposureTime": 70000, "AnalogueGain": 1,  "ColourGains": (1.4,1.5)}) 
        self.image_width = frame_width
        self.image_center = self.image_width // 2 # Calculate the center of the image
        self.cap.start()

    def Capturing(self):
        self.t1 = time.time()  # For measuring FPS
        img = self.CaptureImage()  # Capture a single image frame
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
        #imgHSV = cv2.erode(imgHSV, kernel, iterations=1)
        #imgHSV = cv2.dilate(imgHSV, kernel, iterations=1)
        robotview = img.copy()  # Preserve the original image
        return img, imgHSV, robotview

    def ExportImage(self, WindowName, view, FPS=False):
        if FPS:
            fps = 1.0 / (time.time() - self.t1)  # calculate frame rate
            cv2.putText(view, f'{int(fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen

        cv2.imshow(WindowName, view)

    def findBlack(self, imgHSV):
        # Create masks for the orange color
        #HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
        BlackMask = cv2.inRange(imgHSV, self.color_ranges['black'][0], self.color_ranges['black'][1])
        #contoursBlack, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return BlackMask


    def findWall(self, imgHSV, imgRGB):
        # Create masks for the orange color (wall detection)
        gray = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY)
        ret, WallMask = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)
        #WallMask = cv2.inRange(imgHSV, self.color_ranges['wall'][0], self.color_ranges['wall'][1])

        # Find contours in the mask
        contoursWall1, _ = cv2.findContours(WallMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Check if any contours are found
        if contoursWall1:
            # Find the largest contour
            largest_contour = max(contoursWall1, key=cv2.contourArea)
            
            # Calculate the convex hull of the largest contour
            #hull = cv2.convexHull(largest_contour)
            
            # Create an empty mask and draw the convex hull on it
            filledWallMask = np.zeros_like(WallMask)
            cv2.drawContours(filledWallMask, [largest_contour], -1, (255), thickness=cv2.FILLED)
            
            # Apply Gaussian blur to the filled mask
            filledWallMask = cv2.GaussianBlur(filledWallMask, (9, 9), 2)
            
            # Use the filled mask to extract the wall image
            WallImg = cv2.bitwise_and(imgRGB, imgRGB, mask=filledWallMask)
            
            # Convert the extracted image to grayscale
            WallImgGray = cv2.cvtColor(WallImg, cv2.COLOR_BGR2GRAY)
        else:
            # No contours found, return original image and empty mask
            largest_contour = contoursWall1
            WallImg = np.zeros_like(imgRGB)
            WallImgGray = np.zeros_like(cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY))
            filledWallMask = np.zeros_like(WallMask)

        return WallImg, WallImgGray, filledWallMask, np.array([largest_contour]),gray



    def findMarkers(self, WallImgGray, WallMask):
        _, mask = cv2.threshold(WallImgGray, 150, 255, cv2.THRESH_BINARY_INV)
        markers = cv2.bitwise_and(WallMask, mask)
        _, mask1 = cv2.threshold(markers, 110, 255, cv2.THRESH_BINARY)
        ContoursMarkers, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return ContoursMarkers, mask1



    def findItems(self, imgHSV):
        # Create masks for the orange color
        ItemMask1 = cv2.inRange(imgHSV, self.color_ranges['orange1'][0], self.color_ranges['orange1'][1])
        ItemMask2 = cv2.inRange(imgHSV, self.color_ranges['orange2'][0], self.color_ranges['orange2'][1])
        ItemMask = ItemMask1 | ItemMask2  # Combine masks
        contoursItem, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursItem, ItemMask

    def findShelf(self, imgHSV, area_threshold=10000):
        # Create a mask for the blue color range
        ShelfMask = cv2.inRange(imgHSV, self.color_ranges['blue'][0], self.color_ranges['blue'][1])
        
        # Find contours on the mask
        contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter out small contours by area
        filtered_contours = [cnt for cnt in contoursShelf if cv2.contourArea(cnt) > area_threshold]
        
        return contoursShelf, ShelfMask

    def findLoadingArea(self, imgHSV):
        LoadingAreaMask = cv2.inRange(imgHSV, self.color_ranges['yellow'][0], self.color_ranges['yellow'][1])
        contoursLoadingArea, _ = cv2.findContours(LoadingAreaMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursLoadingArea, LoadingAreaMask

    def findObstacle(self, imgHSV):
        ObstacleMask = cv2.inRange(imgHSV, self.color_ranges['green'][0], self.color_ranges['green'][1])
        contoursObstacle, _ = cv2.findContours(ObstacleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursObstacle, ObstacleMask
    
    # Function to check contour circularity

    def MarkerShapeDetection(self, contoursMarkers, output,image):
        detected = False
        shapeCount = 0
        distances = []  # List to store distances for all detected circles
        bearings = []   # List to store bearings for all detected circles

        for contour in contoursMarkers:
            if (1 < cv2.contourArea(contour) < 50000):  # Area filter
                epsilon = 0.03 * cv2.arcLength(contour, True)
                ShapeContours = cv2.approxPolyDP(contour, epsilon, True)
                #num_vertices = len(ShapeContours)
                num_vertices = 8
                print(num_vertices)
                print(shapeCount, "markers detected")

                #if num_vertices == 4:
                    #shape = "Square"
                if num_vertices in [4, 12]:  # Avoiding conflict with squares
                    shapeCount += 1
                    print(shapeCount, "circle detected")
                    shape = "Circle"

                    # Find the center and radius of the circle
                    (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                    diameter = 2 * radius

                    # Calculate distance and bearing
                    distance = self.GetDistance(diameter, 70)
                    bearing = self.GetBearing(x_center,image)

                    distances.append(distance)  # Store each distance
                    bearings.append(bearing)    # Store each bearing

                    # Draw text and circle on the output image
                    cv2.putText(output, f"Distance: {distance:.2f} cm", 
                                (int(x_center), int(y_center)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (123, 200, 100), 2)
                    cv2.putText(output, f"Circle: {shapeCount}", 
                                (int(x_center), int(y_center) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
                    cv2.circle(output, (int(x_center), int(y_center)), 
                            int(radius), (0, 255, 0), 2)

                    detected = True

        if detected:
            return shapeCount, distances, bearings  # Return list of distances and bearings
        else:
            return 0, [], []  # Return zero shapes, and empty lists for distances and bearings



        
    def GetContoursShelf(self, contours, output, colour, text, Draw=True, min_area=1000):
        detected_centers = []  # List to store the centers of detected shelves
        closest_point = None
        max_y = -1  # Variable to track the point closest to the bottom edge
        
        for contour in contours:
            if cv2.contourArea(contour) > min_area:
                # Calculate the contour's center using moments
                M = cv2.moments(contour)
                if M['m00'] != 0:  # Avoid division by zero
                    center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                else:
                    center = (0, 0)

                if Draw:
                    # Draw the contour
                    cv2.drawContours(output, [contour], -1, colour, 1)

                    # Draw the text at the center of the contour
                    cv2.putText(output, text, center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Store the detected center
                detected_centers.append(center)

                # Find the point closest to the bottom edge within the current contour
                for point in contour:
                    x, y = point[0]  # Extract x and y from the point
                    if y > max_y:  # If this point is closer to the bottom
                        max_y = y  # Update max_y to the current point's y
                        closest_point = (x, y)  # Save the closest point

        # Draw the closest point to the bottom edge, if found
        if closest_point:
            pred_point = cv2.perspectiveTransform(np.float32(closest_point).reshape(-1,1,2), self.homography_matrix)
            real_points = int(100 + pred_point[0][0][1])
            #print(real_points)
            # Draw the point closest to the bottom edge as a red circle
            if Draw:
                # Draw the closest point as a red circle
                cv2.circle(output, closest_point, 5, (0, 0, 255), -1)  # Red circle with radius 5

                # Add text showing the distance next to the dot (closest_point)
                distance_text = f"{real_points:.2f} cm"  # Format the distance to two decimal places
                cv2.putText(output, distance_text, (closest_point[0] + 10, closest_point[1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                

        # Return the list of detected centers, or None if no contours are detected
        if detected_centers:
            return detected_centers
        else:
            return None


    def GetContoursMarkers(self, contours, output, colour, text, Draw=True, min_area=1000):
        detected_centers = []  # List to store the centers of detected shelves
        
        for contour in contours:
            if cv2.contourArea(contour) > min_area:
                # Calculate the contour's center using moments
                M = cv2.moments(contour)
                if M['m00'] != 0:  # Avoid division by zero
                    center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                else:
                    center = (0, 0)

                if Draw:
                    # Draw the contour
                    cv2.drawContours(output, [contour], -1, colour, 1)
                    
                    # Draw the text at the center of the contour
                    cv2.putText(output, text, center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Store the detected center
                detected_centers.append(center)

        # Return the list of detected centers, or None if no contours are detected
        if detected_centers:
            return detected_centers
        else:
            return None
    def GetInfoShelf(self, robotview, ShelfCenters, imgRGB):
        bearing = []
        center = []
        if ShelfCenters is not None:
# Loop through each detected shelf center
            for ShelfCenter in ShelfCenters:
                    x_center, y_center = ShelfCenter
                    cen = (x_center, y_center)
                    # Calculate the bearing (angle) for each shelf
                    ShelfAngle = self.GetBearing(x_center, imgRGB)
                    bearing.append(ShelfAngle)
                    center.append(cen)
                    # Display the angle on the image
                    cv2.putText(robotview, f"A: {int(ShelfAngle)}", (int(x_center), int(y_center)), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 200), 1)
        return center, bearing
    
    
    def GetInfoObject(self, robotview, detected_obstacles, imgRGB):
        distance = []
        bearing = []
        centers = []
        if detected_obstacles is not None:
# Loop through each detected obstacle and process it
            for obstacle in detected_obstacles:
                    x_ObstacleCenter, y_ObstacleCenter, ObHeight, ObWidth = obstacle
                    # Calculate the obstacle's angle and distance
                    ObstacleAngle = self.GetBearing(x_ObstacleCenter, imgRGB)
                    ObstacleDistance = self.GetDistance(ObHeight, 150 )
                    distance.append(ObstacleDistance)
                    bearing.append(ObstacleAngle)
                    centers.append((x_ObstacleCenter, y_ObstacleCenter))
                    # Add the angle and distance information to the image
                    cv2.putText(robotview, f"A: {int(ObstacleAngle)} deg", (int(x_ObstacleCenter), int(y_ObstacleCenter + ObHeight / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
                    cv2.putText(robotview, f"D: {int(ObstacleDistance)} cm", (int(x_ObstacleCenter), int(y_ObstacleCenter)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
        return centers, distance, bearing
    
    def GetInfoMarkers(self, robotview, ContoursMarkers, imgRGB):
        distance = []
        bearing = []
        centers = []

        for contours in ContoursMarkers:
            if cv2.contourArea(contours) > 100:
                (x, y), radius = cv2.minEnclosingCircle(contours)
                center = (int(x), int(y))
                circle_area = np.pi * (radius ** 2)
                contour_area = cv2.contourArea(contours)
                
                # Define the acceptable difference threshold
                area_difference_threshold = 5000  # You can adjust this value

                # Check if the difference between areas is within the threshold
                if abs(contour_area - circle_area) <= area_difference_threshold:
                    MarkerAngle = self.GetBearing(x, imgRGB)
                    MarkerDistance = self.GetDistance(radius * 2, 73)
                    # cv2.putText(robotview, f"A: {int(MarkerAngle)} deg", (int(x), int(y + radius / 2)), 
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
                    # cv2.putText(robotview, f"D: {int(MarkerDistance)} cm", (int(x), int(y)), 
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
                    cv2.circle(robotview, center, int(radius), (255, 255), 2)
                    cv2.drawMarker(robotview, center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=5, thickness=2)
                    cv2.putText(robotview, f"M", (int(x - 6), int(y + radius / 2)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
                    
                    # Store the distance, bearing, and center
                    distance.append(MarkerDistance)
                    bearing.append(MarkerAngle)
                    centers.append(center)

        # Calculate the average center if there are any centers
        if centers:
            avg_x = sum([c[0] for c in centers]) / len(centers)
            avg_y = sum([c[1] for c in centers]) / len(centers)
            avg_center = (int(avg_x), int(avg_y))
            avg_distance = sum(distance) / len(distance)
            avg_bearing = sum(bearing) / len(bearing)
            shape_count = len(centers)
            cv2.drawMarker(robotview, avg_center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=5, thickness=2)
            cv2.putText(robotview, f"A: {int(avg_bearing)} deg", (int(avg_x), int(avg_y + 20)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
            cv2.putText(robotview, f"D: {int(avg_distance)} cm", (int(avg_x), int(avg_y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
            cv2.putText(robotview, f"{shape_count}", (int(avg_x - 10), int(avg_y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        else:
            avg_center = None  # If no centers were found, return None or a default value
            avg_bearing = None
            avg_distance = None
            shape_count = 0

        return avg_center, avg_bearing, avg_distance, shape_count


   
    
    def GetContoursObject(self, contours, output, colour, text, Draw=True):
        detected_objects = []  # List to store detected object info
        
        for contour in contours:
            if cv2.contourArea(contour) > 50:
                x, y, width, height = cv2.boundingRect(contour)
                
                if Draw:
                    cv2.rectangle(output, (x, y), (x + width, y + height), colour, 1)
                    cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Calculate center
                x_center, y_center = x + width // 2, y + height // 2
                
                # Append this object's properties to the list
                detected_objects.append((x_center, y_center, height, width))
        
        # Return list of detected objects
        if detected_objects:
            return detected_objects
        else:
            return None




    def GetContoursMarker(self, contours, output, colour, text, Draw=True):
        detected_objects = []  # List to store detected object info
        
        for contour in contours:
            if cv2.contourArea(contour) > 50:
                # Get the minimum enclosing circle for the contour
                (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                x_center, y_center = int(x_center), int(y_center)
                radius = int(radius)
                
                if Draw:
                    # Draw the circle around the detected object
                    cv2.circle(output, (x_center, y_center), radius, colour, 2)
                    
                    # Draw the text at the center of the circle
                    cv2.putText(output, text, (x_center, y_center - radius - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Append this object's properties to the list (center and radius)
                detected_objects.append((x_center, y_center, radius))
        
        # Return list of detected objects
        if detected_objects:
            return detected_objects
        else:
            return None


    
    def GetDistance(self, width, real_width):
        return (self.focal_length * real_width) / width + 4
    
    def GetBearing(self, x_center,image):
        offset_pixels = x_center - image.shape[1]/ 2
        return (offset_pixels / image.shape[1]) * 70


class CamFrameGrabber:
    # FOV = number of degrees for camera view
    def __init__(self, src, height, width):
        self.camera = picamera2.Picamera2()
        self.width = width
        self.height = height

        # Configure the camera
        config = self.camera.create_video_configuration(main={"format": 'XRGB8888', "size": (height, width)})
        self.camera.configure(config)
        self.camera.set_controls({"ExposureTime": 70000, "AnalogueGain": 1,  "ColourGains": (1.4,1.5)}) 
        self.camera.start()

        self.cameraStopped = False
        self.prev_frame_id = -1
        self.frame_id = 0
        self.currentFrame = np.zeros((height, width, 3), np.uint8)
        #self.currentFrame = self.camera.capture_array() #

    def start(self):
        self.t1 = time.time()
        Thread(target=self.captureImage, args=()).start()
        return self

    def captureImage(self):
        # Continuously capture frames
        while True:
                if self.cameraStopped:
                    return
                # Capture current frame
                self.currentFrame = self.camera.capture_array()
                self.frame_id += 1

    def getCurrentFrame(self):
        self.imgFlip = cv2.resize(self.currentFrame, (410, 308))
        imgRGB = self.imgFlip#cv2.rotate(self.imgFlip, cv2.ROTATE_180)
        imgHSV = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2HSV)  # Convert to HSV
        RobotView = imgRGB.copy()  # Preserve the original image
        return imgRGB, imgHSV, RobotView
    
    def getFrameID(self):
        return self.frame_id
    
    
    def DisplayFrame(self, frame_id, FPS=False, frame=None, frame1=None, frame2=None, frame3=None, frame4=None):
        if frame_id != self.prev_frame_id:
                if FPS:
                    fps = 1.0 / (time.time() - self.t1)  # calculate frame rate
                    self.t1 = time.time()
                    cv2.putText(frame, f'{int(fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen
                
                cv2.imshow('Frame', frame)
                
                if frame1 is not None:
                    cv2.imshow('Frame1', frame1)
                if frame2 is not None:
                    cv2.imshow('Frame2', frame2)
                if frame3 is not None:
                    cv2.imshow('Frame3', frame3)
                if frame4 is not None:
                    cv2.imshow('Frame4', frame4)
                
                self.prev_frame_id = frame_id

        

    def stop(self):
        self.cameraStopped = True
        
    def __del__(self):
        # There is no release method in picamera2, so stop the camera instead
        self.camera.stop()
        cv2.destroyAllWindows()


class MultiImageProcess:
    def __init__(self, src, height, width):
        self.camera = picamera2.Picamera2()
        self.width = width
        self.height = height
        
        # Create the queue inside the class
        self.image_queue = mp.Queue()  # Initialize the queue here
        
        # Configure the camera
        config = self.camera.create_video_configuration(main={"format": 'XRGB8888', "size": (height, width)})
        self.camera.configure(config)
        self.camera.set_controls({"ExposureTime": 70000, "AnalogueGain": 1, "ColourGains": (1.4, 1.5)})
        self.camera.start()

        self.cameraStopped = False
        self.frame_id = 0
        self.currentFrame = np.zeros((height, width, 3), np.uint8)

    def ImageCapturingLoop(self):
        while not self.cameraStopped:
            # Capture current frame
            self.currentFrame = self.camera.capture_array()
            self.frame_id += 1
            imgRGB, imgHSV, RobotView = self.getCurrentFrame()
            self.image_queue.put((imgRGB, imgHSV, RobotView))

    def getCurrentFrame(self):
        imgFlip = cv2.resize(self.currentFrame, (410, 308))
        imgRGB = cv2.rotate(imgFlip, cv2.ROTATE_180)
        imgHSV = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2HSV)  # Convert to HSV
        RobotView = imgRGB.copy()  # Preserve the original image
        return imgRGB, imgHSV, RobotView

    def start(self):
        self.cameraStopped = False
        capture_process = mp.Process(target=self.ImageCapturingLoop)
        capture_process.start()
        return capture_process

    def stop(self):
        self.cameraStopped = True

    def get_queue(self):
        return self.image_queue  # Method to access the queue