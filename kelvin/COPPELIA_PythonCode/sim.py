#!/usr/bin/python
import vision as vs
import time
import cv2
# import the  bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *
def convert_image(img):
	img = np.reshape((np.array(img).astype(np.uint8)), (480,640,3))
	return cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.bayContents= np.random.random_integers(0,5,(6,4,3)) # Random item in each bay
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl # specify a bowl in the bay in shelf 0 
# with x,y coords (3,1) (zero-indexed). Items are {bowl,mug,bottle,soccer,rubiks,cereal}.
sceneParameters.obstacle1_StartingPosition = None
sceneParameters.obstacle2_StartingPosition = None
# SET ROBOT PARAMETERS
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'	# specify if using differential (currently omni is not supported)
vision = vs.VisionModule()
# MAIN SCRIPT
def Rotate(Side):
      if Side == 'left':
            print('Turning Left')
            packerBotSim.SetTargetVelocities(0 , -0.2)
            packerBotSim.SetTargetVelocities(0 , 0.2)  
      elif Side == 'right':
            print('Turning Right')
            packerBotSim.SetTargetVelocities(0 , 0.2)
            packerBotSim.SetTargetVelocities(0 , -0.2)

def Turn45deg(Side):
      if Side == 'left':
            print('Turning Left')
            packerBotSim.SetTargetVelocities(0 , -0.2)
            packerBotSim.SetTargetVelocities(0 , 0.2)  
            time.sleep(15)
            packerBotSim.SetTargetVelocities(0 , 0)
            time.sleep(1)
      elif Side == 'right':
            print('Turning Right')
            packerBotSim.SetTargetVelocities(0 , 0.2)
            packerBotSim.SetTargetVelocities(0 , -0.2)
            time.sleep(15)
            packerBotSim.SetTargetVelocities(0 , 0)
            time.sleep(1)

class RobotStateMachine:
      def __init__(self):
            self.robot_state = 'AWAITING_COMMAND'
      def Update(self, imgRGB, imgHSV, RobotView):
            if self.robot_state == 'AWAITING_COMMAND':
                  pass
            elif self.robot_state == 'MARKERS_SEARCH':
                  print('Searching for markers')
                  WallRGB,  WallImgGray, WallMask = vision.findWall(imgHSV,imgRGB)
                  ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
                  avg_center, avg_bearing, avg_distance, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)
                  Rotate('right') #Rotate the robot to search for markers
                  if (avg_center is not None):
                        if abs(avg_bearing) < 10:
                              self.robot_state = 'APPORACH_BAY'
                              packerBotSim.SetTargetVelocities(0 , 0) 
            elif self.robot_state == 'APPORACH_BAY':
                  print('Approaching the selected bay')
                  WallRGB,  WallImgGray, WallMask = vision.findWall(imgHSV,imgRGB)
                  ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
                  avg_center, avg_bearing, avg_distance, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)
                  if (avg_center is not None): # if the center of the markers is found
                        if avg_distance >= 40:
                              packerBotSim.SetTargetVelocities(0.09 , 0- avg_bearing*0.1)
                        else:
                              packerBotSim.SetTargetVelocities(0, 0)
                              self.robot_state = 'PICKING_ITEM'
                  elif (avg_center is None):
                        packerBotSim.SetTargetVelocities(0 , 0)
            elif self.robot_state == 'PICKING_ITEM':
                  print('Picking Up the Item')
                  Turn45deg('left')
                  packerBotSim.SetTargetVelocities(0.1 , 0) 
                  time.sleep(3)
                  packerBotSim.SetTargetVelocities(0 , 0)
                  packerBotSim.SetTargetVelocities(-0.1 , 0) 
                  time.sleep(2)
                  Turn45deg('left')

                  self.robot_state = 'DELIVER_ITEM'
            elif self.robot_state == 'DELIVER_ITEM':
                  packerBotSim.SetTargetVelocities(0 , 0)





if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C
            try:
                  packerBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
                  packerBotSim.StartSimulator()
                  vision = vs.VisionModule()
                  RobotState = RobotStateMachine()
                  RobotState.robot_state = 'MARKERS_SEARCH' # set the robot initial state to MARKERS_SEARCH
                  while True:
                        imgRGB, imgHSV, RobotView = packerBotSim.Capturing()
                        CenterCoord = vision.draw_crosshair(RobotView)

                        RobotState.Update(imgRGB, imgHSV, RobotView)
                        cv2.imshow('RobotView', RobotView)
                        cv2.waitKey(1)
                               
                        
            except KeyboardInterrupt as e:
                  # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
                  packerBotSim.StopSimulator()