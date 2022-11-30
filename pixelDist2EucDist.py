import numpy as np

def pixelDist2EucDist(xp, yp, h, FOVx=(np.pi/2), FOVy=1.03, xwidth=1280, yheight=720):
    # xp = x coordinate in pixels, yp = y coordinate in pixels. h = 3D distance to point.
    # All angles are in radians
    thetax = (FOVx/xwidth)*(xp-(xwidth/2))
    thetay = (FOVy/yheight)*(yp-(yheight/2))

    x = h*np.sin(thetax)
    y = h*np.sin(thetay)
    z = h*np.cos(thetax)

    pos = [x, y, z]
    return pos