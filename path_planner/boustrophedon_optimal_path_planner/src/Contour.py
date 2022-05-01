import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import matplotlib.image as mpimg
import pyclipper
from math import sqrt
from copy import deepcopy
from Visibility import *

def get_contour_path(start, plot = False, margin = 0):
    input = open("data_files/input.txt", "r")

    perimeter = list()
    firstline = input.readline()
    L = (firstline.strip()).split(' ')
    for i in range(0, len(L) - 1, 2) :
        vertex = (int(L[i]), int(L[i + 1]))
        perimeter.append(vertex)

    pco = pyclipper.PyclipperOffset()
    pco.AddPath(perimeter, pyclipper.JT_SQUARE, pyclipper.ET_CLOSEDPOLYGON) # JT_MITER, JT_ROUND, JT_SQUARE
    solution = pco.Execute(-margin) # dont return the last point, we need to add the first at the end

    assert len(solution) == 1 # if not its means we got 2 contours

    obstacles = list()
    for line in input :
        L = (line.strip()).split(' ')
        polygon = list()
        for i in range(0, len(L) - 1, 2) :
            vertex = (int(L[i]), int(L[i + 1]))
            polygon.append(vertex)
        obstacles.append(polygon)

    for obstacle in obstacles:
        pco = pyclipper.PyclipperOffset()
        pco.AddPath(obstacle, pyclipper.JT_SQUARE, pyclipper.ET_CLOSEDPOLYGON) # JT_MITER, JT_ROUND, JT_SQUARE
        solution_obstacles = pco.Execute(margin) # dont return the last point, we need to add the first at the end
        assert len(solution_obstacles) == 1 # if not its means we got 2 contours
        solution.append(solution_obstacles[0])

    path = []

    for polygon in solution: # for 1 contour we should ennter only 1 time

        start_on_contour = find_closest_start_point(polygon, start) # output = point

        if path: # we enter here after we actually did perimeter (second for loop iteration)
            process_obstacle(start_on_contour, polygon, path, margin)
        else:
            process_perimeter(start_on_contour, polygon, path)

        start = start_on_contour

        ########################################################################
        #### FOR DEBUGGING #####################################################
        ########################################################################

        # posx, posy = zip(*polygon) # convert list of 2 point to 2 tuples  of x and y
        # if plot :
        #     plt.plot(list(posx), list(posy), label = 'polygon')
        #     plt.plot(start_on_contour[0], start_on_contour[1], marker = 'o', color = 'red')
        #     for i in range(5):
        #         plt.annotate(i, (posx[i],posy[i]))
        #     plt.show()
        ########################################################################

    # plotting
    if plot :
        posx, posy = zip(*path) # convert list of 2 point to 2 tuples  of x and y
        plt.plot(list(posx), list(posy), label = 'polygon')
        plt.plot(path[0][0], path[0][1], marker = 'o', color = 'red')
        plt.plot(path[-1][0], path[-1][1], marker = 'x', color = 'blue')
        for i in range(50):
            plt.annotate(i, (posx[i],posy[i]))
        for i in range(len(posx)-10, len(posx)):
            plt.annotate(i, (posx[i],posy[i]))
        plt.show()

    return path

def find_closest_start_point(polygon, start):
    last_dist = 100000000
    for i in range(len(polygon)):
        dist = sqrt((polygon[i][0]-start[0])**2 + (polygon[i][1]-start[1])**2)
        if dist < last_dist:
            start_on_contour = (polygon[i][0], polygon[i][1])
            last_dist = dist

    return start_on_contour

def set_first_element_at_start(polygon, start_on_contour):
    for i in range(len(polygon)):
        if polygon[0][0] != start_on_contour[0] or polygon[0][1] != start_on_contour[1]: # we roll until the first point is at the start_point
            polygon = np.roll(polygon,1,axis=0).tolist()
        else:
            break
    return polygon

def add_discretized_path_between_each_point(polygon, path):
    for i in range(len(polygon)-1):
        start_x = polygon[i][0]
        start_y = polygon[i][1]
        end_x = polygon[i+1][0]
        end_y = polygon[i+1][1]

        dist = sqrt((end_x-start_x)**2 + (end_y-start_y)**2)

        # dicretize if the distance is big
        if dist > 2:
            nb_points = int(dist) - 1 # we want a point at each pixel

            x_path = np.linspace(start_x, end_x, nb_points).tolist()
            y_path = np.linspace(start_y, end_y, nb_points).tolist()
            path_points_discretize = []

            for i in range(0,len(x_path)-1): # dont want a point at the end
                path_points_discretize.append((x_path[i], y_path[i]))

            path_points_discretize = list(zip(x_path, y_path))
            path.extend(path_points_discretize)

def process_perimeter(start_on_contour, polygon, path):
    polygon = set_first_element_at_start(polygon, start_on_contour) # polygon = list of lists
    polygon.append(polygon[0]) # append at the end the first element
    add_discretized_path_between_each_point(polygon, path) # path = list of points

def process_obstacle(start_on_contour, polygon, path, margin):
    index_to_add_transition = len(path)

    polygon = set_first_element_at_start(polygon, start_on_contour) # polygon = list of lists

    cost, path_transition = shortest_path(start = path[-1], goal = start_on_contour, plot = False, margin = margin)

    polygon.append(polygon[0]) # append at the end the first element
    add_discretized_path_between_each_point(polygon, path) # path = list of points

    # Add the transition path between end of path before adding this new contour and the start of this contour
    path[index_to_add_transition:index_to_add_transition] = path_transition # extend path with transition
    # Add the transition path between the start of this contour and end of path before adding this new contour
    path.extend(path_transition[::-1])

if __name__ == "__main__" :
    image = mpimg.imread("data_files/map.jpg")
    H, W = image.shape[0], image.shape[1]
    # print(H,W)
    my_dpi = 100
    fig = plt.figure(figsize = (W / my_dpi, H / my_dpi), dpi = my_dpi, frameon = False)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    ax.imshow(image, aspect = "auto")

    start = (254, 324)
    start = (-21, 1717)
    plt.plot(start[0], start[1], marker = 'o', color = 'orange')

    path = get_contour_path(start, plot = True, margin = 5)
    #print(path)
    print("len(path)", len(path))
    print("path[0]: ",path[0])
