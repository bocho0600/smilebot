def move_robot(linear_velocity, angular_velocity):
    '''
    @brief Move the robot based on linear and angular velocity
    @param linear_velocity    Speed of the robot's forward/backward motion (positive for forward, negative for backward)
    @param angular_velocity   Rate of rotation (positive for right turn, negative for left turn)
    '''
    # Convert velocities to motor speed ranges
    max_speed = 100  # Assuming 100 is the maximum speed
    left_motor_speed = linear_velocity - angular_velocity
    right_motor_speed = linear_velocity + angular_velocity

    # Ensure the speeds are within the allowable range
    left_motor_speed = max(min(left_motor_speed, max_speed), -max_speed)
    right_motor_speed = max(min(right_motor_speed, max_speed), -max_speed)

    # Determine the movement direction based on speed values
    if left_motor_speed > 0:
        board.motor_movement([board.M1], board.CW, abs(left_motor_speed))
    elif left_motor_speed < 0:
        board.motor_movement([board.M1], board.CCW, abs(left_motor_speed))
    else:
        board.motor_stop([board.M1])
    
    if right_motor_speed > 0:
        board.motor_movement([board.M2], board.CW, abs(right_motor_speed))
    elif right_motor_speed < 0:
        board.motor_movement([board.M2], board.CCW, abs(right_motor_speed))
    else:
        board.motor_stop([board.M2])
    
    # Print the status for debugging
    print(f"Left motor speed: {left_motor_speed}")
    print(f"Right motor speed: {right_motor_speed}")

# Example usage
move_robot(50, 10)  # Move forward with a slight right turn
move_robot(-50, -10)  # Move backward with a slight left turn
