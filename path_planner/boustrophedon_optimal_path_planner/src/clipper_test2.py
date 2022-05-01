import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import matplotlib.image as mpimg
import pyclipper

if __name__ == "__main__" :

    # perimeter = [(x,y) for x,y in zip(posx,posy)] # convert back to points

    original = [(-21, 1717), (-21, 435), (1554, 436), (1554, 1638), (1464, 1638), (1464, 1472), (1071, 1472), (1071, 1558), (978, 1558), (978, 1125), (1092, 993), (1268, 993), (1393, 868), (1393, 632), (1268, 508), (1028, 508), (908, 632), (907, 811), (736, 1001), (752, 1558), (647, 1558), (647, 1472), (224, 1472), (224, 1707)] #(253, 956), (208, 572), (547, 744)
    # original = [[348, 257], [364, 148], [362, 148], [326, 241], [295, 219], [258, 88], [440, 129], [370, 196], [372, 275]]
    # original.append(original[0])
    posx, posy = zip(*original) # convert list of 2 point to 2 tuples  of x and y
    print(list(posx))
    print(list(posy))

    fig, ax = plt.subplots()
    plt.scatter(posx, posy, label = 'original')

    pco = pyclipper.PyclipperOffset()
    pco.AddPath(original, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON) # JT_MITER, JT_ROUND, JT_SQUARE
    solution = pco.Execute(-7.0) # dont return the last point, we need to add the first at the end

    print("-------------------------------------------")
    print("POLYGONS")
    for polygon in solution:
        print("polygon:")
        print(polygon)
        print("-------------------------------------------")
        polygon.append(polygon[0])
        posx, posy = zip(*polygon) # convert list of 2 point to 2 tuples  of x and y

        plt.plot(list(posx), list(posy), label = 'polygon')

    # plt.show()

    original = [(253, 917), (253, 749), (400, 749), (400, 917)] #(253, 956), (208, 572), (547, 744)
    pco = pyclipper.PyclipperOffset()
    pco.AddPath(original, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON) # JT_MITER, JT_ROUND, JT_SQUARE
    solution = pco.Execute(-7.0) # dont return the last point, we need to add the first at the end

    print("-------------------------------------------")
    print("POLYGONS")
    for polygon in solution:
        print("polygon:")
        print(polygon)
        print("-------------------------------------------")
        polygon.append(polygon[0])
        posx, posy = zip(*polygon) # convert list of 2 point to 2 tuples  of x and y

        plt.plot(list(posx), list(posy), label = 'polygon')

    plt.show()

""" Result (2 polygons, see image below):
[[[365, 260], [356, 254], [363, 202]], [[425, 133], [365, 191], [371, 149], [370, 145], [368, 142], [364, 141], [362, 141], [358, 142], [355, 145], [322, 230], [301, 215], [268, 98]]]
"""
