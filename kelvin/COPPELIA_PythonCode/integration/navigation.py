import numpy as np
import math
import matplotlib.pyplot as plt

import math
import numpy as np
import matplotlib.pyplot as plt

class NavigationModule:

    def __init__(self):
        self.paused_update = False

    def clip_deg_fov(self, angle, FOV):
        if angle <= 0:
            angle = 0
        elif angle >= FOV:
            angle = FOV
        return angle   

    def deg_to_rad(self, deg):
        return math.radians(deg)

    def rad_to_deg(self, rad):
        return math.degrees(rad)

    def compute_attractor_field(self, goal_deg):
        CAMERA_FOV = 66
        attraction_field = np.zeros(CAMERA_FOV + 1)
        attraction_field[goal_deg] = 1
        gradient = 1 / 30
        for angle in range(0, int((CAMERA_FOV / 2 + 1))):
            attraction_field[self.clip_deg_fov(goal_deg - angle, CAMERA_FOV)] = 1 - angle * gradient
            attraction_field[self.clip_deg_fov(goal_deg + angle, CAMERA_FOV)] = 1 - angle * gradient
        return attraction_field

    def compute_repulsive_field(self, obstacles):
        WORKER_WIDTH_SCALE = 0.15  # m
        CAMERA_FOV = 66
        
        repulsive_field = np.zeros(CAMERA_FOV + 1)
        
        if obstacles:
            for obs in obstacles:
                obs_range = obs[0]
                obs_bearing = obs[1]

                # only consider obstacles that are less than 1 metre away
                if obs_range < 1:
                    obs_deg = int(self.rad_to_deg(obs_bearing) + CAMERA_FOV / 2)
                    
                    obs_width_rad = 2 * math.atan(WORKER_WIDTH_SCALE / obs_range)
                    obs_width_deg = int(self.rad_to_deg(obs_width_rad))
                    
                    obs_effect = max(0, 1 - min(1, obs_range - WORKER_WIDTH_SCALE * 2))

                    repulsive_field[obs_deg] = obs_effect
                    
                    for angle in range(1, obs_width_deg + 1):
                        repulsive_field[self.clip_deg_fov(obs_deg - angle, CAMERA_FOV)] = max(
                            repulsive_field[self.clip_deg_fov(obs_deg - angle, CAMERA_FOV)], obs_effect)
                        repulsive_field[self.clip_deg_fov(obs_deg + angle, CAMERA_FOV)] = max(
                            repulsive_field[self.clip_deg_fov(obs_deg + angle, CAMERA_FOV)], obs_effect)

        return repulsive_field

    def calculate_goal_velocities(self, goal_position, obstacles, draw=False):
        # MAX_ROBOT_VEL = 0.05
        # MAX_ROBOT_ROT = 0.2
        MAX_ROBOT_VEL = 0.1
        MAX_ROBOT_ROT = 0.4
        GOAL_P = 0.5
        ROTATIONAL_BIAS = 0.5
        CAMERA_FOV = 66
        
        # Compute bearing to goal
        goal_rad = goal_position['bearing']
        goal_deg = int(self.clip_deg_fov(self.rad_to_deg(goal_rad) + CAMERA_FOV / 2, CAMERA_FOV))
        
        # Compute both attractive and repulsive field maps
        nav_state = {}
        nav_state['attractive_field'] = self.compute_attractor_field(goal_deg)
        nav_state['repulsive_field'] = self.compute_repulsive_field(obstacles)
        
        # Compute residual map (difference between attractive and repulsive fields)
        nav_state['residual_field'] = np.maximum(0, nav_state['attractive_field'] - nav_state['repulsive_field'])
    
        # Find heading angle by finding the index of the max value in the residual field
        heading_angle = np.argmax(nav_state['residual_field'])
        goal_error = self.deg_to_rad(heading_angle) - self.deg_to_rad(CAMERA_FOV / 2)
    
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


def calculate_avg_range_bearing(rowMarkerRangeBearing):
    # Initialize lists to store valid ranges and bearings
    valid_ranges = []
    valid_bearings = []

    # Loop through each marker
    for marker in rowMarkerRangeBearing:
        if marker is not None:
            valid_ranges.append(marker[0])
            valid_bearings.append(marker[1])

    # Calculate average only if there are valid markers
    avg_range = sum(valid_ranges) / len(valid_ranges) if valid_ranges else None
    avg_bearing = sum(valid_bearings) / len(valid_bearings) if valid_bearings else None

    # Return the averages as a dictionary or tuple
    return {'range': avg_range, 'bearing': avg_bearing}


def count_valid_markers(rowMarkerRangeBearing):
    # Count markers that are not None
    valid_markers_count = sum(1 for marker in rowMarkerRangeBearing if marker is not None)
    return valid_markers_count
