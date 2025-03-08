from G16Modules.Vision import VisionModule
from G16Modules.Globals import SCREEN_HEIGHT, SCREEN_WIDTH
import numpy as np

if __name__ == "__main__":

    test_cases = np.array([ [SCREEN_WIDTH/2, SCREEN_HEIGHT/2],
                            [0, 0],
                            [SCREEN_WIDTH, 0],
                            [0, SCREEN_HEIGHT],
                            [SCREEN_WIDTH, SCREEN_HEIGHT],
                            [100, 100],
                            [250,88]])


    VisionModule.calculate_projection_transform()
    
    p1 = VisionModule.project_to_ground(test_cases)
    p2 = VisionModule.project_to_screen(p1)


    print(" IN ")
    print(test_cases)

    print(" Project ")
    print(p1)

    print(" OUT ")
    print(p2)

    print(" ERROR ")
    error = p2 - test_cases
    print(error)