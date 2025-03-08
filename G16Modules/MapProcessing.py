import numpy as np
import cv2

class ArenaMap:
    def __init__(self):
        # Let 0.0 be the packing station corner with all coordinates positive

        width = 2.0
        height = 2.0
        self.size = (width, height)
        self.wall_lines = np.array([[[0., 0.],
                                     [0., height],
                                     [width, height],
                                     [width, 0.],
                                     [0.,0.]]]) # clockwise cause inside is in bounds

        # Boundaries anticlockwise cause inside is out of bounds
        shelf_length = 1.04
        shelf_depth = 0.125
        sl = []

        shelf_points = []

        shelf_1 = np.array([[0.,height-shelf_length],
                            [shelf_depth,height-shelf_length],
                            [shelf_depth,height],
                            [0.,height],
                            [0.,height-shelf_length]])
        
        shelf_2 = np.array([[-shelf_depth,height-shelf_length],
                            [+shelf_depth,height-shelf_length],
                            [+shelf_depth,height],
                            [-shelf_depth,height],
                            [-shelf_depth,height-shelf_length]])
        
        sl.append(shelf_1)
        sl.append(shelf_2 + np.array([width/3., 0.]))
        sl.append(shelf_2 + np.array([2.*width/3., 0.]))
        sl.append(shelf_1 + np.array([width-shelf_depth,0.]))
        sl = np.array(sl)

        shelf_points = sl[:, :-1, :].reshape(16, 2)
        self.shelf_points = shelf_points

        marker_locations = []
        marker_locations.append([width, 1*width/6])
        marker_locations.append([width, 3*width/6]) 
        marker_locations.append([width, 5*width/6]) 
        self.marker_locations = np.array(marker_locations)

        packing_station_size = 0.585
        packing_station_flat = 0.29
        pl = np.array([[[0.,0.],
                        [packing_station_size,0.],
                        [packing_station_size,packing_station_flat],
                        [packing_station_flat,packing_station_size],
                        [0.,packing_station_size],
                        [0.,0.]]])


        self.shelf_lines = np.flip(sl, 2) # I had it the wrong way around
        self.packing_lines = pl
        self.width = width
        self.height = height
    
    def draw_arena(self, image_width):
        image = np.zeros((image_width, image_width, 3), np.uint8)
        
        # Ground, Packing Station, Shelves
        scale = image_width/self.width
        image = cv2.fillPoly(image, pts=(self.wall_lines * scale).astype(np.int32), color=(100,100,100))
        image = cv2.fillPoly(image, pts=(self.packing_lines * scale).astype(np.int32), color=(0,255,255))
        image = cv2.fillPoly(image, pts=(self.shelf_lines * scale).astype(np.int32), color = (255, 0,0))
        
        self.arena_image = image.copy()

        # Show image
        return image
    
    def closestSegment(self, point, type, visiblefrom=None):
        segments = None
        match type:
            case "shelf":
                segments = self.shelf_lines
            case "packing":
                segments = self.packing_lines
            case "wall":
                segments = self.wall_lines
            case _:
                segments = np.concatenate([self.shelf_lines, self.wall_lines], 0)
                return None
            
        # Dims: 0=line; 1=start/end; 2=x/y 
        segments = np.array([segments[:, :-1, :], segments[:, 1:, :]]).transpose((1,2,0,3)).reshape((-1, 2, 2))
        ds = segments[:, 1, :] - segments[:, 0, :] # end - start
        cs = point - segments[:, 0, :] # point - start

        if visiblefrom is not None:
            vs = visiblefrom - segments[:, 0, :]
            # Only visible (facing towards)
            mask = np.cross(ds, vs) > 0
            segments = segments[mask]
            ds = ds[mask]
            cs = cs[mask]

        # ts is the point from 0 to 1 along the segment which is closest
        ts = (cs * ds).sum(1) / (ds*ds).sum(1)
        ts = np.clip(ts, 0, 1)

        # ps is the projections from point onto each segment (closest point on each segment)
        ps = segments[:, 0, :] + np.array([ts,ts]).T * ds

        # vector from projection (closest point on each segment) to the point
        ls = ps - point 

        # square distances
        qs = (ls*ls).sum(1) 

        closest_segment = segments[np.argmin(qs)]
        distance_to = np.sqrt(np.min(qs))
        
        return closest_segment, distance_to

    
         

    def getForce(self, point):
        segments = np.concatenate([self.shelf_lines, self.wall_lines], 0)

         # Dims: 0=line; 1=start/end; 2=x/y 
        segments = np.array([segments[:, :-1, :], segments[:, 1:, :]]).transpose((1,2,0,3)).reshape((-1, 2, 2))
        ds = segments[:, 1, :] - segments[:, 0, :] # end - start
        cs = point - segments[:, 0, :] # point - start

        # ts is the point from 0 to 1 along the segment which is closest
        ts = (cs * ds).sum(1) / (ds*ds).sum(1)
        ts = np.clip(ts, 0, 1)

        # ps is the projections from point onto each segment (closest point on each segment)
        ps = segments[:, 0, :] + np.array([ts,ts]).T * ds

        # vector from projection (closest point on each segment) to the point
        ls = point - ps 

        # distances
        qs = np.sqrt((ls*ls).sum(1))

        def force_fun(x):
            x = np.abs(x)
            margin = 0.2 # metres to stay away from it
            alpha = 3
            x[x>margin] = margin
            return ((margin - x) / margin)**alpha

        f = force_fun(qs)
        force = np.c_[f,f] * ls / np.c_[qs,qs]


        return force.sum(0)
        


map = None

def onMouseClick(event, x, y, flags, param):
    global target_position
    scale = 512/2.0
    if flags & 1:
        point = np.array([x, y]) / scale

        path = [point]
        ds = 0.01 # m
        for i in range(50):
            R = (target_position - point)
            force = 10 * map.getForce(point) + R
            #force = force / np.sqrt(np.sqrt((force*force).sum()))
            point = point.copy() + ds * force
            path.append(point)
            if (R*R).sum() < 0.05**2:
                break
        
        path = np.array(path)
        disp_im = cv2.drawMarker(image.copy(), (target_position*scale).astype(np.int32), (0,0,0), 15)
        disp_im = cv2.polylines(disp_im, [(path*scale).astype(np.int32)], False, (0, 0, 255), 2)
      
        cv2.imshow("Arena", disp_im)
    elif flags & 2:
        target_position = np.array([x, y]) / scale
        disp_im = cv2.drawMarker(image.copy(), (target_position*scale).astype(np.int32), (0,0,0), 15)
        cv2.imshow("Arena", disp_im)

target_position = np.array([.1,.1])

if __name__ == "__main__":
    map = ArenaMap()
    image = map.draw_arena(512)

    cv2.imshow("Arena", image)
    cv2.setMouseCallback("Arena", onMouseClick)
    while(1):
        k = cv2.waitKey(0) & 0xFF
        if k == 27:
            break
        elif k == ord('q'):
            break
