from .Globals import *
from .COPPELIA.warehousebot_lib import *
import cv2

class SimSpecific:

    # Specific should implement:
	# get_frame
	# set_velocity
	# start
	# update
	# stop

    color_ranges = {
            'floor': (np.array([0, 0, 80]), np.array([0, 0, 135])),
            'wall': (np.array([0, 0, 146]), np.array([30, 1, 255])),
            'blue': (np.array([1, 165, 50]), np.array([10, 200, 120])),
            'black': (np.array([0, 0, 0]), np.array([0, 0, 0])),
            'yellow': (np.array([99, 216, 130]), np.array([99, 217, 187])),
            'green': (np.array([40, 90, 0]), np.array([70, 255, 180])),
            'orange1':(np.array([  115, 200,  60]), np.array([  130, 220, 160])),
            'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
		}


    if is_hitl: # hardware in the loop
        coppelia_server_ip = '192.168.235.29'
    else:
        coppelia_server_ip = '127.0.0.1'

    # SET SCENE PARAMETERS
    sceneParameters = SceneParameters()
    sceneParameters.bayContents= np.random.random_integers(0,5,(6,4,3)) # Random item in each bay
    sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl # specify a bowl in the bay in shelf 0 
    # with x,y coords (3,1) (zero-indexed). Items are {bowl,mug,bottle,soccer,rubiks,cereal}.

    # SET ROBOT PARAMETERS
    robotParameters = RobotParameters()
    robotParameters.driveType = 'differential'	# specify if using differential (currently omni is not supported)

    packerBotSim = None

    @classmethod
    def get_image(cls):
        _, img = cls.packerBotSim.GetCameraImage()
        if img is None:
            return None, None, None
        
        # img = np.reshape((np.array(img).astype(np.uint8)), (480,640,3))
        img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
        img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT), cv2.INTER_NEAREST)
        imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        robotview = img #.copy()

        return img, imgHSV, robotview

    
    @classmethod
    def set_velocity(cls, fwd, rot):
        cls.packerBotSim.SetTargetVelocities(fwd, -rot)
    
    @classmethod
    def start(cls):
        cls.packerBotSim = COPPELIA_WarehouseRobot(cls.coppelia_server_ip, cls.robotParameters, cls.sceneParameters)
        cls.packerBotSim.StartSimulator()
        cls.packerBotSim.SetCameraPose(0.1, 0.1, 0)

    @classmethod
    def update(cls):
        cls.packerBotSim.UpdateObjectPositions() # needs to be called once at the end of the main code loop

    @classmethod
    def end(cls):
        cls.packerBotSim.StopSimulator()

    @classmethod
    def leds(cls, mask):
        pass

    @classmethod
    def lifter_set(cls, h):
        pass
