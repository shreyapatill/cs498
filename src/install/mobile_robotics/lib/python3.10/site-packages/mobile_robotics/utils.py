import numpy as np
import math

def quaternion_from_euler(ai, aj, ak):
    '''
    Arguments:
        ai, aj, ak: Euler angles in RPY order (Roll, Pitch and Yaw) (float)
    Returns:
        q: The corresponding quaternion in format [qx, qy, qz, qw] (python list)
    '''
    # Calculate half angles
    roll_half = ai * 0.5
    pitch_half = aj * 0.5
    yaw_half = ak * 0.5

    # Calculate sine and cosine of half angles
    cr = math.cos(roll_half)
    sr = math.sin(roll_half)
    cp = math.cos(pitch_half)
    sp = math.sin(pitch_half)
    cy = math.cos(yaw_half)
    sy = math.sin(yaw_half)

    # Calculate quaternion components using ZYX (yaw-pitch-roll) convention
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    q = [qx, qy, qz, qw]

    return q 


def lonlat2xyz(lat, lon, lat0, lon0): 
    # WGS84 ellipsoid constants:
    a = 6378137
    b = 6356752.3142
    e = math.sqrt(1-b**2/a**2)
    
    x = a*math.cos(math.radians(lat0))*math.radians(lon-lon0)/math.pow(1-e**2*(math.sin(math.radians(lat0)))**2,0.5)
    y = a*(1 - e**2)*math.radians(lat-lat0)/math.pow(1-e**2*(math.sin(math.radians(lat0)))**2,1.5)
    
    return x, y # x and y coordinates in a reference frame with the origin in lat0, lon0
