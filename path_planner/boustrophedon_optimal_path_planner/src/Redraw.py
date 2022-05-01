# redraw the map according to the vertex manually annotated on map_original.jpg
# map_original.jpg -> map.jpg

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import matplotlib.image as mpimg
from math import cos, sin

def redraw(W, H) :
    input = open("data_files/input.txt", "r")

    perimeter = list()
    firstline = input.readline()
    L = (firstline.strip()).split(' ')
    for i in range(0, len(L) - 1, 2) :
        vertex = (int(L[i]), int(L[i + 1]))
        perimeter.append(vertex)

    polygons = list()
    for line in input :
        L = (line.strip()).split(' ')
        # print('L',L)
        polygon = list()
        for i in range(0, len(L) - 1, 2) :
            vertex = (int(L[i]), int(L[i + 1]))
            polygon.append(vertex)
        # print('polygon',polygon)
        polygons.append(polygon)

    my_dpi = 100

    fig = plt.figure(figsize = (W / my_dpi, H / my_dpi), dpi = my_dpi, frameon = False)

    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)

    image = np.full((H, W, 3), 0, dtype = np.uint8)
    # print(image[5,:,0])
    ax.imshow(image, aspect = "auto")
    # ax.scatter(x0,y0, c='r', lw=5)
    # ax.annotate('C', fontsize=10, xy=(x0, y0))

    ax.add_patch(patches.Polygon(perimeter, color = "white")) # first patch is the perimeter so it should be white
    for polygon in polygons :
        # print('here',polygon)
        ax.add_patch(patches.Polygon(polygon, color = "black"))

    fig.savefig("data_files/map.jpg", dpi = my_dpi)

if __name__ == "__main__" :
    W, H = 1000, 1000
    redraw(W, H)
    plt.show()
