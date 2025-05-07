import numpy as np

class gtrack_cartesian_position():
    def __init__(self, x=0., y=0., z=0.):
        self.posX = x
        self.posY = y
        self.posZ = z

def sph2cart(point):
    sinAzim = np.sin(point.azimuth)
    cosAzim = np.cos(point.azimuth)
    sinElev = np.sin(point.elevation)
    cosElev = np.cos(point.elevation)
    x_pos = point.range * cosElev * sinAzim
    y_pos = point.range * cosElev * cosAzim
    z_pos = point.range * sinElev

    return gtrack_cartesian_position(x_pos, y_pos, z_pos)

DEFAULT_BOX = [(0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)]
class gtrack_boundaryBox():
    def __init__(self, left, right, near, far, bottom, top):
        self.left = left
        self.right = right
        self.near = near
        self.far = far
        self.bottom = bottom
        self.top = top

def isPointInsideBox(point, box):
    if (point.x > box.left) and (point.x < box.right) and \
       (point.y > box.near) and (point.y < box.far) and \
       (point.z > box.bottom) and (point.z < box.top):
        return 1
    else:
        return 0

def boxWorld2Sensor(box, sensor_position, sensor_orientation):
    translation = np.array([sensor_position.x, sensor_position.y, sensor_position.z])
    
    new_min = np.array([box.left, box.near, box.bottom]) - translation
    new_max = np.array([box.right, box.far, box.top]) - translation
    #only supports elevation rotation
    elev = sensor_orientation.elevTilt
    rotation_matrix = np.array([
        [1.0,           0,            0],
        [0.0,  np.cos(-elev),  np.sin(-elev)],
        [0.0, -np.sin(-elev),  np.cos(-elev)]
    ]) 

    new_min = (rotation_matrix @ new_min.T).T
    new_max = (rotation_matrix @ new_max.T).T

    return gtrack_boundaryBox(new_min[0], new_max[0], new_min[1], new_max[1], new_min[2], new_max[2])

# GTRACK Measurement point
class gtrack_measurementPoint():
    def __init__(self):
        self.range = 0.
        self.azimuth = 0.
        self.elevation = 0.
        self.doppler = 0.

# GTRACK Measurement variances
class gtrack_measurementVariance():
    def __init__(self):
        self.rangeVar = 0
        self.azimuthVar = 0
        self.elevationVar = 0
        self.dopplerVar = 0