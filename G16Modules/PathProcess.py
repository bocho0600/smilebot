from threading import Thread, Lock
from .Globals import *
from .Vision import VisionModule
import time
from math import sin, cos
import numpy as np

class PathProcess:

    thread = None
    running = True
    path = []
    seg = 0

    fwd = 0
    rot = 0
    fwdlen = None
    rotlen = None
    completed = True

    # points in global coordinates that we want to keep track of
    # e.g. shelf corners to avoid
    # nav will want to use them in coordinates relative to the robot though (to add and read)
    # Also when we change target or re-localize we should also either update or clear the points.
    track_points = None
    track_points2 = None

    integration_lock = Lock()


    @classmethod
    def motorLoop(cls):
        tl = time.time()
        time.sleep(0.01)
        while cls.running:
            tn = time.time()
            delta = tn-tl
            tl = tn

            cls.integrate()

            c_fwd, c_rot = cls.currentSpeed()
            if cls.fwdlen is not None:
                cls.fwdlen -= abs(c_fwd) * delta
            if cls.rotlen is not None:
                cls.rotlen -= abs(c_rot) * delta
            
            # Note: if the segments are very short and we move fast, we maybe should be skipping some segments but this will not happen in the current code
            # Todo if we go below 0, carry that over to the next segment and continue skipping any number of segments until the extra distance is accounted for
            if (cls.fwdlen is not None and cls.fwdlen <= 0) or (cls.rotlen is not None and cls.rotlen <= 0):
                cls.next_seg()
            
            Specific.set_velocity(cls.fwd, cls.rot)
            # print(f"{cls.fwd:.2f}, {cls.rot:.2f}")

            time.sleep(0.01) # ~ 100 Hz
            # print(f"Motorloop {1.0/delta:.2f} fps")

    @classmethod 
    def next_seg(cls, inc=True):
        if inc:
            cls.seg += 1

        if cls.seg < len(cls.path):
            cls.fwd, cls.rot, cls.fwdlen, cls.rotlen = cls.path[cls.seg]
        else:
            cls.fwd = 0
            cls.rot = 0
            cls.fwdlen = None
            cls.rotlen = None
            cls.completed = True

    @classmethod
    def new_path(cls, newpath):
        cls.path = newpath.copy()
        cls.seg = 0
        cls.completed = False
        cls.next_seg(False)
        Specific.set_velocity(cls.fwd, cls.rot)


    @classmethod
    def currentSpeed(cls):
        # todo implement Specific.odometry()
        return cls.fwd, cls.rot


    track_x = None
    track_y = None
    track_h = None
    last_integrated = None

    @classmethod
    def stop_integrating(cls):
        with cls.integration_lock:
            cls.track_x = None
            cls.track_y = None
            cls.track_h = None
            cls.last_integrated = None

    @classmethod
    def localize(cls, x, y, h, update_tracked_points=True):
        
        if update_tracked_points and cls.last_integrated is not None:
            lx, ly, _ = cls.get_current_track()
            dx = x - lx
            dy = y - ly
            cls.track_points = cls.track_points + np.array([[dx, dy]])
            cls.track_points2 = cls.track_points2 + np.array([[dx, dy]])

        with cls.integration_lock:
            cls.track_x = x
            cls.track_y = y
            cls.track_h = h
            cls.last_integrated = time.time()

    @classmethod
    def integrate(cls):
        if cls.last_integrated is None:
            return
    
        # this might be called from either thread. If problems arise use a lock
        with cls.integration_lock:
            now = time.time()
            delta = now - cls.last_integrated

            fwd, rot = cls.currentSpeed()
            rot = -rot # AARGG
            h0 = cls.track_h
            if rot == 0:
                h1 = h0
                dx = fwd * cos(h0) * delta
                dy = fwd * sin(h0) * delta
            else:
                h1 = h0 + rot * delta
                dx = fwd/rot * (sin(h1) - sin(h0))
                dy = fwd/rot * (cos(h0) - cos(h1))

            cls.track_x += dx
            cls.track_y += dy
            cls.track_h = h1

            cls.last_integrated = now

    @classmethod
    def transform_world_to_bot(cls, coords):
        if coords is None:
            return np.array([])
        elif len(coords)<1:
            return coords
        
        x,y,h = cls.get_current_track()
        rc = coords.T - np.array([[x],[y]])
        rc = np.matmul(np.array([[cos(h),sin(h)],[sin(h),-cos(h)]]), rc)
        return rc.T

    @classmethod
    def transform_bot_to_world(cls, coords):
        if coords is None:
            return np.array([])
        elif len(coords)<1:
            return coords
        
        x,y,h = cls.get_current_track()
        rc = np.matmul(np.array([[cos(h),sin(h)],[sin(h),-cos(h)]]), coords.T)
        rc = rc + np.array([[x],[y]])
        return rc.T

    @classmethod
    def add_tracked_point_relative(cls, point, bank=0):
        if bank==0:
            if cls.track_points is None or len(cls.track_points) < 1:
                cls.track_points = cls.transform_bot_to_world(np.array([point]))
            else:
                cls.track_points = np.r_[cls.track_points, cls.transform_bot_to_world(np.array([point]))]
        else:
            if cls.track_points2 is None or len(cls.track_points2) < 1:
                cls.track_points2 = cls.transform_bot_to_world(np.array([point]))
            else:
                cls.track_points2 = np.r_[cls.track_points2, cls.transform_bot_to_world(np.array([point]))]

    @classmethod
    def get_tracked_points_relative(cls, bank=0):
        cls.integrate()
        if bank==0:
            return cls.transform_world_to_bot(cls.track_points)
        else:
            return cls.transform_world_to_bot(cls.track_points2)



    @classmethod
    def clear_tracked_points(cls, bank=0):
        if bank==0:
            cls.track_points = None
        else:
            cls.track_points2 = None
    
    @classmethod
    def get_current_track(cls):
        cls.integrate()
        return cls.track_x, cls.track_y, cls.track_h

    @classmethod
    def Start(cls):
        cls.thread = Thread(target=cls.motorLoop)
        cls.thread.start()
    
    @classmethod
    def End(cls):
        cls.running = False
        if cls.thread is not None and cls.thread.is_alive():
            cls.thread.join()
    
    @classmethod
    def set_velocity(cls, fwd, rot):
        cls.new_path([(fwd, rot, None, None)])