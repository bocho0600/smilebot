#!/usr/bin/python

# Import the bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *
import numpy as np  # Ensure numpy is imported
import math
import matplotlib.pyplot as plt

# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.bayContents = np.random.randint(0, 5, (6, 4, 3))  # Random item in each bay
sceneParameters.bayContents[0, 3, 1] = warehouseObjects.bowl  # Specify a bowl in the bay in shelf 0


# SET ROBOT PARAMETERS
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'  # Specify if using differential (currently omni is not supported)

def clip_deg_fov(angle, FOV):
    if angle <= 0:
        angle = 0
    elif angle >= FOV:
        angle = FOV
    return angle   

def deg_to_rad(deg):
    return math.radians(deg)

def rad_to_deg(rad):
    return math.degrees(rad)

def compute_attractor_field(goal_deg):
    CAMERA_FOV = 60
    attraction_field = np.zeros(CAMERA_FOV + 1)
    attraction_field[goal_deg] = 1
    gradient = 1 / 30
    for angle in range(0, int((CAMERA_FOV / 2 + 1))):
        attraction_field[clip_deg_fov(goal_deg - angle, CAMERA_FOV)] = 1 - angle * gradient
        attraction_field[clip_deg_fov(goal_deg + angle, CAMERA_FOV)] = 1 - angle * gradient
    return attraction_field

def compute_repulsive_field(obstacles):
    WORKER_WIDTH_SCALE = 0.15  # m
    CAMERA_FOV = 60
    
    repulsive_field = np.zeros(CAMERA_FOV + 1)
    
    if obstacles is not None:
        for obs in obstacles:
            obs_range = obs[0]
            obs_bearing = obs[1]

            # Only consider obstacles that are less than 1 metre away
            if obs_range < 1:
                obs_deg = int(rad_to_deg(obs_bearing) + CAMERA_FOV / 2)
                obs_width_rad = 2 * math.atan(WORKER_WIDTH_SCALE / obs_range)
                obs_width_deg = int(rad_to_deg(obs_width_rad))
                obs_effect = max(0, 1 - min(1, obs_range - WORKER_WIDTH_SCALE * 2))

                repulsive_field[obs_deg] = obs_effect
                
                for angle in range(1, obs_width_deg + 1):
                    repulsive_field[clip_deg_fov(obs_deg - angle, CAMERA_FOV)] = max(repulsive_field[clip_deg_fov(obs_deg - angle, CAMERA_FOV)], obs_effect)
                    repulsive_field[clip_deg_fov(obs_deg + angle, CAMERA_FOV)] = max(repulsive_field[clip_deg_fov(obs_deg + angle, CAMERA_FOV)], obs_effect)

    return repulsive_field

def calculate_goal_velocities(goal_position, obstacles, draw=False):
    MAX_ROBOT_VEL = 0.05
    MAX_ROBOT_ROT = 0.2
    GOAL_P = 0.5
    ROTATIONAL_BIAS = 0.5
    CAMERA_FOV = 60
    
    # Compute bearing to goal
    goal_rad = goal_position['bearing']
    goal_deg = int(clip_deg_fov(rad_to_deg(goal_rad) + CAMERA_FOV / 2, CAMERA_FOV))
    
    # Compute both attractive and repulsive field maps
    nav_state = {}
    nav_state['attractive_field'] = compute_attractor_field(goal_deg)
    nav_state['repulsive_field'] = compute_repulsive_field(obstacles)
    
    # Compute residual map (difference between attractive and repulsive fields)
    nav_state['residual_field'] = np.maximum(0, nav_state['attractive_field'] - nav_state['repulsive_field'])
 
    # Find heading angle by finding the index of the max value in the residual field
    heading_angle = np.argmax(nav_state['residual_field'])
    goal_error = deg_to_rad(heading_angle) - deg_to_rad(CAMERA_FOV / 2)
   
    # Balance the amount of forward velocity vs rotational velocity if the robot needs to turn
    desired_rot_vel = min(MAX_ROBOT_ROT, max(-MAX_ROBOT_ROT, goal_error * GOAL_P))
    desired_vel = MAX_ROBOT_VEL * (1.0 - ROTATIONAL_BIAS * abs(desired_rot_vel) / MAX_ROBOT_ROT)
    
    nav_state['rotational_vel'] = desired_rot_vel
    nav_state['forward_vel'] = desired_vel
    
    # If draw is True, plot the fields
    if draw:
        plt.figure(2)
        plt.clf()
        
        plt.subplot(3, 1, 1)
        plt.plot(nav_state['attractive_field'])
        plt.title('Attractive Field')
        
        plt.subplot(3, 1, 2)
        plt.plot(nav_state['repulsive_field'])
        plt.title('Repulsive Field')
        
        plt.subplot(3, 1, 3)
        plt.plot(nav_state['residual_field'])
        plt.plot([heading_angle, heading_angle], [0, 1], 'r')
        plt.plot([goal_deg, goal_deg], [0, 1], 'g')
        plt.title('Residual Field')
        
        plt.draw()
        plt.pause(0.001)
    
    return nav_state

# MAIN SCRIPT
if __name__ == '__main__':
    try:
        warehouseBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
        warehouseBotSim.StartSimulator()
        goal_bay_position = [0.875, 0.625, 0.375, 0.1] # bay positions in the row
        found_row = False
        action = {}
        robot_state = 'START'
        goal_position = {'range': None, 'bearing': None}  # Initialize goal position

        while True:
            itemsRB, packingBayRB, obstaclesRB, rowMarkerRB, shelvesRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.items, warehouseObjects.packingBay, warehouseObjects.obstacles, warehouseObjects.row_markers, warehouseObjects.shelves])    
            if robot_state == 'START':
                robot_state = 'SEARCH_FOR_ROW'
                action['forward_vel'] = 0
                action['rotational_vel'] = 0

            elif robot_state == 'SEARCH_FOR_ROW':
                for row_index in range(0, 3):  
                    if rowMarkerRB[row_index] is not None: # Check if row marker is detected
                        found_row = True
                        #break  # Exit loop after finding the first marker
                action['forward_vel'] = 0
                action['rotational_vel'] = -0.1 # Rotate to search for row marker
                
                if found_row:
                    robot_state = 'MOVE_DOWN_ROW'

            elif robot_state == 'MOVE_DOWN_ROW':
                found_row = False
                for row_index in range(0, 3): # from 0 to 2
                    if rowMarkerRB[row_index] is not None: # Check if row marker is detected 
                        found_row = True
                        goal_position['range'] = rowMarkerRB[row_index][0]
                        goal_position['bearing'] = rowMarkerRB[row_index][1]         
                        break  

                if found_row:
                    print('MarkersRange and Bearing:', goal_position)
                    action = calculate_goal_velocities(goal_position, obstaclesRB)
                else:
                    robot_state = 'SEARCH_FOR_ROW'
                    action['forward_vel'] = 0
                    action['rotational_vel'] = 0
                
                if goal_position['range'] is not None and (goal_position['range'] - goal_bay_position[2] < 0.1):
                    robot_state = 'AT_BAY'
                    action['forward_vel'] = 0
                    action['rotational_vel'] = 0

            elif robot_state == 'AT_BAY':
                print('At the bay')
                action['forward_vel'] = 0
                action['rotational_vel'] = 0

            warehouseBotSim.SetTargetVelocities(action.get('forward_vel', 0), action.get('rotational_vel', 0))
            warehouseBotSim.UpdateObjectPositions()

    except KeyboardInterrupt:
        # Stop simulator so it doesn't need manual stopping in VREP
        warehouseBotSim.StopSimulator()
