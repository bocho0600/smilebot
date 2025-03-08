import cv2
#import picamera2
import numpy as np
import time

# MADE BY KELVIN LE, QUT EGB320 GROUP16 
# StudentID: n11429984
# Email: minhnguyen.le@qut.edu.au

class VisionModule:
      color_ranges = {
            'floor': (np.array([0, 0, 80]), np.array([0, 0, 103])),
            'wall': (np.array([0, 0, 146]), np.array([30, 1, 255])),
            'blue': (np.array([3, 171, 54]), np.array([3, 175, 112])),
            'black': (np.array([0, 0, 0]), np.array([0, 0, 0])),
            'yellow': (np.array([99, 216, 130]), np.array([99, 217, 187])),
            'green': (np.array([40, 90, 0]), np.array([70, 255, 180])),
            'orange1': (np.array([5, 150, 150]), np.array([20, 255, 255])),
            'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
      }
      # color_ranges = {
      #       'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
      #       'yellow': (np.array([20, 85, 0]), np.array([32  , 255, 255])),
      #       'blue': (np.array([90, 126, 58]), np.array([130, 255, 245])),
      #       'green': (np.array([40, 90, 0]), np.array([70, 255, 180])),
      #       'orange1': (np.array([5, 150, 150]), np.array([20, 255, 255])),
      #       'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
      #       'black': (np.array([0, 0, 43]), np.array([179, 55, 109]))
      # }
      def CaptureImage(self):
            frame = cap.capture_array()          # capture a single image frame
            frame = cv2.flip(frame, 0) # OPTIONAL: Flip the image vertically
            return frame
      
      # def initialize_camera(self, frame_height=320*2, frame_width=240*2, format='XRGB8888'):
      #       #create a camera object
      #       cap = picamera2.Picamera2()
      #       #print the different camera resolutions/modes 
      #       #the sensor can be configured for
      #       #print(cap.sensor_modes) #OPTIONAL

      #       #set a specific configuration, smaller resolution will be faster
      #       #however will have a cropped field of view
      #       #consider a balance between higher resolution, field of view and frame rate
      #       frameHeight, frameWidth = 320*2, 240*2
      #       config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(frameHeight,frameWidth)})
      #       cap.configure(config)

      #       #start the camera
      #       cap.start()

      #       return cap
      
      def findItems(self, img):
            imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV) #Convert to HSV
            # Create masks for the orange color
            ItemMask1 = cv2.inRange(imgHSV, self.color_ranges['orange1'][0], self.color_ranges['orange1'][1])
            ItemMask2 = cv2.inRange(imgHSV, self.color_ranges['orange2'][0], self.color_ranges['orange2'][1])
            ItemMask = ItemMask1 | ItemMask2  # Combine masks
            contoursItem, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return contoursItem, ItemMask

      def findShelf(self, img):
            imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV) #Convert to HSV
            ShelfMask = cv2.inRange(imgHSV, self.color_ranges['blue'][0], self.color_ranges['blue'][1])
            contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return contoursShelf, ShelfMask

      def findLoadingArea(self,img):
            imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV) #Convert to HSV
            LoadingAreaMask = cv2.inRange(imgHSV, self.color_ranges['yellow'][0], self.color_ranges['yellow'][1])
            contoursLoadingArea, _ = cv2.findContours(LoadingAreaMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return contoursLoadingArea, LoadingAreaMask
      
      def findObstacle(self,img):
            imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            ObstacleMask = cv2.inRange(imgHSV, self.color_ranges['green'][0], self.color_ranges['green'][1])
            contoursObstacle, _ = cv2.findContours(ObstacleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return contoursObstacle, ObstacleMask
      
      def findFloor(self,img):
            imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            FloorMask = cv2.inRange(imgHSV, self.color_ranges['floor'][0], self.color_ranges['floor'][1])
            contoursFloor, _ = cv2.findContours(FloorMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return contoursFloor, FloorMask

      def DrawContours(self, contours, output, colour, text ):
            detected = False
            for contour in contours:
                  if cv2.contourArea(contour) > 500:
                        x, y, width, height = cv2.boundingRect(contour)
                        cv2.rectangle(output, (x, y), (x + width, y + height), colour, 2)
                        cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        detected = True
            if detected:
                  x1, y1, x2, y2 = x, y, x + width, y + height
                  return output, x1, y1, x2, y2
            else:
                  return output, None, None, None, None
      


if __name__ == "__main__":

      vision = VisionModule()
      cap = vision.initialize_camera()

      while(1):

            t1 = time.time()                     # for measuring fps
            
            img = vision.CaptureImage()          # capture a single image frame (should not modify in advance)
            robotview = img.copy() # preserve the original image
            
            contoursItem, ItemMask = vision.findItems(img)
            contoursShelf, ShelfMask = vision.findShelf(img)
            contoursLoadingArea, LoadingAreaMask = vision.findLoadingArea(img)
            Mask = ItemMask | ShelfMask

            robotview, x1, y1, x2, y2 = vision.DrawContours(contoursItem, robotview, (0, 255, 0), "Item")
            robotview, x1, y1, x2, y2 = vision.DrawContours(contoursShelf, robotview, (0, 0, 255), "Shelf")
            
            result = cv2.bitwise_and(robotview, robotview, mask=Mask)


            fps = 1.0/(time.time() - t1)         # calculate frame rate
            cv2.putText(robotview, f'{int(fps)}', (20,30), cv2.FONT_HERSHEY_TRIPLEX ,0.7,(255,255,100),2) # Display the FPS on the screen
            cv2.imshow("Identification", result)     # Display the obtained frame in a window called "CameraImage"  
            cv2.imshow("RobotView", robotview)     # Display the obtained frame in a window called "CameraImage"
            if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                  break
            

      cap.close()