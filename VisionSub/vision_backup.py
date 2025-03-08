import cv2
import picamera2
import numpy as np
from threading import Thread
import vision as vs
import time
from vision import VisionModule
from RP2040 import I2C
from mobi import MobilityModule

# Servo S3 is 6 kg/cm3
# Servo S2 is 3 kg/cm3

def main():
      FRAME_WIDTH = 820
      FRAME_HEIGHT = 616
      cam = vs.CamFrameGrabber(src=0, height=FRAME_WIDTH, width=FRAME_HEIGHT)
      cam.start()
      vision = vs.VisionModule()
      try:
            while True:

                  imgRGB, imgHSV, RobotView = cam.getCurrentFrame()
                  frame_id = cam.getFrameID()
                  CenterCoord = vision.draw_crosshair(RobotView)
                  
                  

                  #Find contours for the shelves
                  contoursShelf, ShelfMask = vision.findShelf(imgHSV)
                  # Get the detected shelf centers
                  ShelfCenters = vision.GetContoursShelf(contoursShelf, RobotView, (0, 0, 255), "S", Draw=True)
                  ShelfCenter, ShelfBearing = vision.GetInfoShelf(RobotView, ShelfCenters, imgRGB)

                  contoursLoadingBay, LoadingBayMask = vision.findLoadingArea(imgHSV)
                  LoadingBayCenters = vision.GetContoursShelf(contoursLoadingBay, RobotView, (0, 255, 0), "L", Draw=True)
                  LoadingBayCenter, LoadingBayBearing = vision.GetInfoShelf(RobotView, LoadingBayCenters, imgRGB)
                  
                  # Detect obstacles in the HSV image
                  contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
                  # Get the list of detected obstacles' centers and dimensions
                  detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)
                  ObsCenters, ObsDistance, ObsBearing = vision.GetInfoObject(RobotView, detected_obstacles, imgRGB)


                  
                  WallRGB,  WallImgGray, WallMask,contoursWall1, GrayScale_Image = vision.findWall(imgHSV,imgRGB)
                  ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
                  avg_center, avg_bearing, avg_distance, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)
                  
                  # if shape_count < 3:
                  #       MobilityModule.Move(0, 20)
                  # else :
                  #       MobilityModule.Move(0, 0)

                  # gray = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY)
                  # ret, WM = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)
                  # inverted_loading_bay_mask = cv2.bitwise_not(LoadingBayMask)
                  # masked_wall = cv2.bitwise_and(WM, inverted_loading_bay_mask)
                  
                  cam.DisplayFrame(frame_id, FPS=True, frame=RobotView, frame1 = WallMask) # Display the frame with the detected objects.
                  # Break the loop if 'q' is pressed
                  if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

      except KeyboardInterrupt:
            # Handle when user interrupts the program
            print("Stopping camera capture...")

      # Stop the camera and close windows
      cam.stop()
      cv2.destroyAllWindows()


if __name__ == "__main__":
    
    main()