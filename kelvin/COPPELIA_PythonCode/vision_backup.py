import cv2
import picamera2
import numpy as np
from threading import Thread
import vision as vs
import time

from G16Modules.Vision import VisionModule


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
                  if contoursShelf:
                        points = VisionModule.combine_contour_points(contoursShelf, False)
                        points, projected_floor = VisionModule.project_and_filter_contour(points)
                        if points is not None and points.shape[0] > 3:
                              dist_map = VisionModule.get_dist_map(points, projected_floor)
                              closest_idx = np.argmin(dist_map[:,0])
                              points_index = np.argmax(points[:, 0] >= closest_idx)
                              closest_shelf_dist = dist_map[closest_idx, 0] * 100
                              closest_point_x = points[points_index, 0]
                              closest_point_y = points[points_index, 1]
                              cv2.drawMarker(RobotView, (closest_point_x, closest_point_x), (0, 255, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=1)
                              cv2.putText(RobotView,  f"D: {int(closest_shelf_dist)}", (closest_point_x, closest_point_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)



                  # Detect obstacles in the HSV image
                  contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
                  # Get the list of detected obstacles' centers and dimensions
                  detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)
                  ObsCenters, ObsDistance, ObsBearing = vision.GetInfoObject(RobotView, detected_obstacles, imgRGB)
                  if ObsDistance is not None:
                        print(ObsDistance)

                  
                  WallRGB,  WallImgGray, WallMask = vision.findWall(imgHSV,imgRGB)
                  ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
                  avg_center, avg_bearing, avg_distance, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)


                  cam.DisplayFrame(frame_id, FPS=True, frame=RobotView) # Display the frame with the detected objects.
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
