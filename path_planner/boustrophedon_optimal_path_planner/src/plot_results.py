import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import matplotlib.image as mpimg
import pyclipper
from math import sqrt

if __name__ == "__main__" :

    test_case = 1
    width = 1
    color = 'red'

    image = mpimg.imread(str(test_case) + "/map.jpg")
    H, W = image.shape[0], image.shape[1]
    # print(H,W)
    my_dpi = 100
    fig = plt.figure(figsize = (W / my_dpi, H / my_dpi), dpi = my_dpi, frameon = False)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    ax.imshow(image, aspect = "auto")

    input = open(str(test_case) + "/fullpath.txt", "r")

    path = list()
    firstline = input.readline()
    L = (firstline.strip()).split(' ')
    for i in range(0, len(L) - 1, 2) :
        vertex = (int(L[i]), int(L[i + 1]))
        path.append(vertex)

    posx, posy = zip(*path) # convert list of 2 point to 2 tuples  of x and y

    # plt.plot(posx, posy, color = color, linewidth = width, zorder = 4)
    # plt.show()

    from matplotlib.animation import FuncAnimation

    line, = ax.plot([], [])

    print('duration: {} ms.'.format(len(posx)))

    def func_animate(i):
        line.set_data(posx[:i], posy[:i])
        return line,

    ani = FuncAnimation(fig,
                        func_animate,
                        frames=len(posx),
                        interval=1) # delay between frame

    ani.save(r'animation.gif', fps=60)
    plt.show()
