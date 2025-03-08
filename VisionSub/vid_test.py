import cv2
import numpy as np
import vision as vs
import math

def is_edge_line(pt1, pt2, image_width, image_height):
    """Check if the line touches the edges of the frame."""
    x1, y1 = pt1
    x2, y2 = pt2

    # If either point lies on the image boundary (edges), consider it an edge line
    if (x1 == 0 or x1 == image_width-1 or y1 == 0 or y1 == image_height-1) and (x2 == 0 or x2 == image_width-1 or y2 == 0 or y2 == image_height-1):
        return True
    return False

def draw_custom_lines(image, contours):
    lines = []  # To store the lines and their lengths
    image_height, image_width = image.shape[:2]  # Get image dimensions

    # Store the points of horizontal and vertical lines separately
    horizontal_lines = []
    vertical_lines = []

    for contour in contours:
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Only consider contours with at least 4 points
        if len(approx) >= 4:
            # Collect line segments and their lengths
            for i in range(len(approx)):
                pt1 = tuple(approx[i][0]) # Point1 of the line
                pt2 = tuple(approx[(i + 1) % len(approx)][0])  # Wrap around to the first point

                # Eliminate lines that touch the edges of the image
                if is_edge_line(pt1, pt2, image_width, image_height):
                    continue  # Skip this line if it touches the edge

                # Calculate the Euclidean distance (length of the line)
                length = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
                
                # Append (pt1, pt2, length) to the list
                lines.append((pt1, pt2, length))

    # Sort the lines by length in descending order
    lines = sorted(lines, key=lambda x: x[2], reverse=True)

    # Only take the top 5 longest lines
    longest_lines = lines[:7]

    # Now draw the longest lines and calculate their angles
    for pt1, pt2, length in longest_lines:
        # Calculate the angle of the line relative to the horizontal
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1] #Take the coordinates of the vector
        angle_radians = math.atan2(dy, dx) # Calculate the angle in radians
        angle_degrees = abs(math.degrees(angle_radians)) # Convert to degrees and take the absolute value

        if angle_degrees > 45 and angle_degrees < 135:  # Vertical Line
            cv2.line(image, pt1, pt2, (0, 255, 0), 2)
            vertical_lines.append((pt1, pt2))  # Store vertical lines
        elif angle_degrees < 45 or angle_degrees > 135:  # Horizontal Line
            cv2.line(image, pt1, pt2, (255, 0, 0), 2)
            horizontal_lines.append((pt1, pt2))  # Store horizontal lines

        # Optionally, display the angle on the image at the midpoint
        midpoint = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
        cv2.putText(image, f"{angle_degrees:.1f} deg", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # Check for mutual points between horizontal lines and vertical lines (white points)
    for h_line in horizontal_lines:
        for v_line in vertical_lines:
            mutual_point = get_mutual_point(h_line, v_line)
            if mutual_point:
                cv2.circle(image, mutual_point, 5, (255, 255, 255), -1)  # Draw white circle for mutual points

    # Check for mutual points between two horizontal lines (black points)
    for i in range(len(horizontal_lines)):
        for j in range(i + 1, len(horizontal_lines)):
            mutual_point = get_mutual_point(horizontal_lines[i], horizontal_lines[j])
            if mutual_point:
                cv2.circle(image, mutual_point, 5, (0, 0, 0), -1)  # Draw black circle for mutual points

    return image

def get_mutual_point(line1, line2):
    """Check if two lines have any mutual points."""
    pt1_1, pt1_2 = line1
    pt2_1, pt2_2 = line2

    # Check if the endpoints of the lines match
    if pt1_1 == pt2_1 or pt1_1 == pt2_2:
        return pt1_1
    elif pt1_2 == pt2_1 or pt1_2 == pt2_2:
        return pt1_2
    return None






def main():
    # Path to the image file
      image_path = 'VisionSub/videos/img.jpg'  # Replace with the path to your image
      
      # Load the image
      image = cv2.imread(image_path)
      #print(image.shape)
      image = image[700:2048, 0:2731]
      #image = cv2.flip(image, 1)
      if image is None:
            print(f"Error: Could not load image from {image_path}")
            return

      FRAME_WIDTH = 820
      FRAME_HEIGHT = 616

      # Resize the image to the desired resolution (if necessary)
      image = cv2.resize(image, (FRAME_WIDTH, FRAME_HEIGHT))

      # Initialize vision module
      vision = vs.VisionModule()

      try:
            # Convert the image to different color spaces
            imgRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            RobotView = image.copy()


            # Draw crosshair at the center
            CenterCoord = vision.draw_crosshair(RobotView)

            # Find contours for the shelves
            contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            ShelfCenters = vision.GetContoursShelf(contoursShelf, RobotView, (0, 0, 255), "S", Draw=True)
            ShelfCenter, ShelfBearing = vision.GetInfoShelf(RobotView, ShelfCenters, imgRGB)
            draw_custom_lines(RobotView, contoursShelf)
     


            # Display the processed image
            cv2.imshow("Processed Image", RobotView)

            # Wait indefinitely for a key press to close the window
            cv2.waitKey(0)

      except KeyboardInterrupt:
            print("Stopping image processing...")

    # Close all OpenCV windows
      cv2.destroyAllWindows()

if __name__ == "__main__":
      main()
