import numpy as np

class gtrack_cartesian_position():
    def __init__(self, x=0., y=0.):
        self.posX = x
        self.posY = y

def sph2cart(point):
    x_pos = point.range * np.sin(point.angle)
    y_pos = point.range * np.cos(point.angle)

    return gtrack_cartesian_position(x_pos, y_pos)

DEFAULT_BOX = [(0, 0, 0, 0), (0, 0, 0, 0)]
# GTRACK Box Structure
class gtrack_boundaryBox():
    def __init__(self, left, right, bottom, top):
        self.left = left
        self.right = right
        self.bottom = bottom
        self.top = top

def isPointInsideBox(point, box):
    if (point.x > box.left) and (point.x < box.right) and (point.y > box.bottom) and (point.y < box.top):
        return 1
    else:
        return 0
    
def boxWorld2Sensor(box, sensor_position, sensor_orientation):
    #current boxWorld2Sensor only supports elevation rotation and vertical translation so nothing is done for 2D
    return box
    
# GTRACK Measurement point
class gtrack_measurementPoint():
    def __init__(self):
        self.range = 0.
        self.angle = 0.
        self.doppler = 0.
        self.snr = 0.


# GTRACK Measurement variances
class gtrack_measurementVariance():
    def __init__(self):
        self.rangeVar = 0
        self.angleVar = 0
        self.dopplerVar = 0