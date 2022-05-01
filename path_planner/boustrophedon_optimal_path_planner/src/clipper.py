import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import matplotlib.image as mpimg
import pyclipper

if __name__ == "__main__" :

    # perimeter = [(x,y) for x,y in zip(posx,posy)] # convert back to points

    original = [(348, 257), (364, 148), (362, 148), (326, 241), (295, 219), (258, 88), (440, 129), (370, 196), (372, 275)]
    # original = [[348, 257], [364, 148], [362, 148], [326, 241], [295, 219], [258, 88], [440, 129], [370, 196], [372, 275]]
    original.append(original[0])
    posx, posy = zip(*original) # convert list of 2 point to 2 tuples  of x and y
    print(list(posx))
    print(list(posy))

    fig, ax = plt.subplots()
    plt.plot(posx, posy, label = 'original')

    pco = pyclipper.PyclipperOffset()
    pco.AddPath(original, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON) # JT_MITER, JT_ROUND, JT_SQUARE
    solution = pco.Execute(-7.0) # dont return the last point, we need to add the first at the end

    for polygon in solution:
        print(polygon)
        polygon.append(polygon[0])
        posx, posy = zip(*polygon) # convert list of 2 point to 2 tuples  of x and y

        plt.plot(list(posx), list(posy), label = 'polygon')

    plt.show()
""" Result (2 polygons, see image below):
[[[365, 260], [356, 254], [363, 202]], [[425, 133], [365, 191], [371, 149], [370, 145], [368, 142], [364, 141], [362, 141], [358, 142], [355, 145], [322, 230], [301, 215], [268, 98]]]
"""
