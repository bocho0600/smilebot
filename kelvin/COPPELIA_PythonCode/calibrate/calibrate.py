import cv2
import numpy as np
from sys import argv
import os

class ObjectType:
    def __init__(self, name):
        self.name = name
        self.inside = set()
        self.outside = set()
        self.thresh_min = None
        self.thresh_max = None
        self.non_empty = False

    def add_point_in(self, point):
        print(point)
        self.inside.add((point[0],point[1],point[2]))
    
    def add_point_out(self, point):
        self.outside.add((point[0],point[1],point[2]))

    def reset(self):
        self.inside = set()
        self.outside = set()
        self.thresh_min = None
        self.thresh_max = None
        self.non_empty = False
    
    def update_thresh(self):
        arr = np.array(list(self.inside))
        if len(arr) > 0:
            self.thresh_min = arr.min(0)
            self.thresh_max = arr.max(0)
            self.non_empty = True
        else:
            self.thresh_min = None
            self.thresh_max = None
            self.non_empty = False
        return self.get_thresh()
    
    def get_thresh(self):
        return (self.thresh_min, self.thresh_max)


current_object = ObjectType("default")
imagesRGB = []
imagesHSV = []
fnames = []
images_dictHSV = {}

def show_mask():
    for imageRGB,imageHSV,fname in zip(imagesRGB,imagesHSV,fnames):
        thresh = current_object.update_thresh()
        if current_object.non_empty:
            mask = cv2.inRange(imageHSV, thresh[0], thresh[1])
            cv2.imshow("out:"+fname, cv2.bitwise_and(imageRGB, imageRGB, mask=mask))
        else:
            cv2.imshow("out:"+fname, np.zeros(imageRGB.shape))

        

def onMouseClick(event, x, y, flags, param):
    if flags:
        #print(event, x, y, flags, param)

        if flags & 1:
            #print(f"IN: {images_dict[param][y,x]}")
            current_object.add_point_in(images_dictHSV[param][y,x])
        elif flags & 2:
            #print(f"OUT: {images_dict[param][y,x]}")
            current_object.add_point_out(images_dictHSV[param][y,x])
    
    if event == 4 or event == 5:
        print(f"Recalculate thresh for {current_object.name}")
        show_mask()
        
def display_thresholds(types):
    print("color_ranges = {")
    for object_type in types:
        if object_type.non_empty:
            print(f"\t\t'{object_type.name}': (np.array({str([o for o in object_type.thresh_min])}), np.array({str([o for o in object_type.thresh_max])})),")
    print("}")
   
    
    

if __name__ == "__main__":
    img_path = "calibrate" #input("Relative path to images: ./")

    fnames = [name for name in os.listdir(f"./{img_path}") if name.endswith(".png")]

    for name in fnames:

        new_imageRGB = cv2.imread(f"./{img_path}/{name}")
        imagesRGB.append(new_imageRGB)

        new_imageHSV = cv2.cvtColor(new_imageRGB, cv2.COLOR_RGB2HSV)
        imagesHSV.append(new_imageHSV)

        images_dictHSV[name] = new_imageHSV

    print(f"Found {len(imagesRGB):d} images")

    T_floor = ObjectType("floor")
    T_wall = ObjectType("wall")
    T_shelf = ObjectType("shelf")
    T_marker = ObjectType("marker")
    T_packing = ObjectType("packing")


    for image,fname in zip(imagesRGB,fnames):
        cv2.imshow(fname, image)
        cv2.setMouseCallback(fname, onMouseClick, fname)
        cv2.imshow("out:"+fname, image)
    
    types = [T_floor, T_wall, T_shelf, T_marker, T_packing] 
    current_object = T_floor
    while(1):
        k = cv2.waitKey(0) & 0xFF
        if k == 27:
            break
        elif k >= ord('1') and k <= ord('5'):
            current_object = types[k - ord('1')]
            print(f"Selected: {current_object.name} for editing")
            show_mask()
        elif k == ord('r'):
            current_object.reset()
            print(f"Reset thresholds for {current_object.name}")
            show_mask()
        elif k == ord('q'):
            display_thresholds(types)
            break
        


