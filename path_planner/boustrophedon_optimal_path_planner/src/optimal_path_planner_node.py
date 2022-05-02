from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib import patches
import random
import pickle
from Redraw import *
from Decomposition import *
from Boustrophedon_path import *
from Visibility import *
from Contour import *
import os
from math import pi
from pprint import pprint
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path


# 1 : 640 * 480
# 2 : 1200 * 1200
# 3 : 1080 * 1080
# 4 : 1280 * 720
# 5 : 1080 * 1080
# Figure3 : 480 * 480
# 6 : 1080 * 1080

dpi = 2000 # should be the same as in permimeter.py
W, H = dpi, dpi

# option
# generate_decomposition, generate_distance_matrix, generate_visiting_order : only required for a new map
generate_input_file = 1
generate_map = 1
generate_decomposition = 1
generate_path_length = 1
generate_visiting_order = 1

save_fig = 0
show_cell_id = 0

# variable
robot_radius = 12 # distance between line and distance to borders, transition doesnt always work, workaround: lower robot_radius for shortest_path
robot_radius_shortest_path = 4
boundary_color = 'blue'
boundary_width = 2
intra_path_color = 'silver'
inter_path_color = 'dimgray'

path_width = 2
marker_size = 6

def figure() :
    if not save_fig :
        plt.pause(0.5)

def display_cell(cell, cell_id) :
    # boundary

    for y in cell.left :
        plt.plot(cell.min_x, y, marker = 's', color = boundary_color, markersize = boundary_width, zorder = 1)

    for y in cell.right :
        plt.plot(cell.max_x, y, marker = 's', color = boundary_color, markersize = boundary_width, zorder = 1)

    ceiling_x = sorted(cell.ceiling)
    for i in range(len(ceiling_x)) :
        if i == 0 or abs(cell.ceiling[ceiling_x[i]] - cell.ceiling[ceiling_x[i - 1]]) <= 1 :
            plt.plot(ceiling_x[i], cell.ceiling[ceiling_x[i]], marker = 's', color = boundary_color, markersize = boundary_width, zorder = 1)
        else :
            plt.plot([ceiling_x[i - 1], ceiling_x[i]], [cell.ceiling[ceiling_x[i - 1]], cell.ceiling[ceiling_x[i]]], color = boundary_color, linewidth = boundary_width, zorder = 1)

    floor_x = sorted(cell.floor)
    for i in range(len(floor_x)) :
        if i == 0 or abs(cell.floor[floor_x[i]] - cell.floor[floor_x[i - 1]]) <= 1 :
            plt.plot(floor_x[i], cell.floor[floor_x[i]], marker = 's', color = boundary_color, markersize = boundary_width, zorder = 1)
        else :
            plt.plot([floor_x[i - 1], floor_x[i]], [cell.floor[floor_x[i - 1]], cell.floor[floor_x[i]]], color = boundary_color, linewidth = boundary_width, zorder = 1)

    if show_cell_id :
        plt.text(cell.center[0] + 0.5, cell.center[1] + 0.5, str(cell_id), color = 'brown', weight = 'bold',
        fontsize = 18, horizontalalignment = 'center', verticalalignment = 'center', zorder = 3)
    figure()

def display_cells(cells) :
    for cell_id in range(1, total_cells_number + 1) :
        cell = cells[cell_id]
        display_cell(cell, cell_id)

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
    qy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
    return qx, qy



class path_planner_from_perimeter:

	def __init__(self):
		# Creates a unique node
		rospy.init_node('path_planner_from_perimeter', anonymous=True)

		# A subscriber to an array of PoseStamped which is called when a message is received
		self.path_subscriber = rospy.Subscriber("/path_planner/perimeter_points", Path, self.callback_path)
        self.translation_publisher = rospy.Publisher('/path_planner/translation', Point, queue_size=10)

    def cartesian_to_dpi(input_path): # margin should be negative for obstacle
        min_x = 0
        min_y = 0
        for pose in input_path.poses) :
            if pose.pose.position.x < min_x:
                min_x = pose.pose.position.x
            if pose.pose.position.y < min_y:
                min_y = pose.pose.position.y

        translation_point = Point()
        translation_point.x = abs(min_x)
        translation_point.y = abs(min_y)
        self.translation_publisher.publish(translation_point)

        pco = pyclipper.PyclipperOffset()
        pco.AddPath(perimeter, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON) # JT_MITER, JT_ROUND, JT_SQUARE
        solutions = pco.Execute(margin) # dont return the last point, we need to add the first at the end
        # These params should be the same as in perimeter.py
        translation = {'x':translation_point.x , 'y':translation_point.y}
        dpi = 2000
        range_meters = 20
        min_fitted = 0
        max_fitted = range_meters

        dpi_based_path = input_path
        for i in range(0, len(dpi_based_path.poses)) :
            x = input_path.poses[i].pose.position.x
            y = input_path.poses[i].pose.position.y

        for i in range(len(solutions[0])):
            x_norm = int(solutions[0][i][0])/dpi
            y_norm = int(solutions[0][i][1])/dpi
            # denormalize
            x = (x_norm - 1)*(max_fitted - min_fitted) + max_fitted - translation['x'] # we remove the translation we added before
            y = (y_norm - 1)*(max_fitted - min_fitted) + max_fitted - translation['y']
            point = (x, y)
            perimeter_cartesian.append(point)

        perimeter_cartesian.append(perimeter_cartesian[0])
        return dpi_based_path

    def input_file(self, path, rotation = 0.3):
		first_pose = input_path.poses[0] # first point should be centroid
        x = first_pose.pose.position.x
        input = open("data_files/input_raw.txt", "r") # IMPORTANT: put the centroid at the beggining of this input file
        x0 = 200
        y0 = 200

        perimeter = list()

        for i in range(0, len(input_path.poses)) :
            x = input_path.poses[i].pose.position.x
            y = input_path.poses[i].pose.position.y
            # we get the centroid at the beginning
            if i == 0:
                x0 = int(x)
                y0 = int(y)
            else:
                vertex = (int(x), int(y))
                perimeter.append(vertex)

        polygons = list() #TODO take this from another ros topic
        # for line in input :
        #     L = (line.strip()).split(' ')
        #     polygon = list()
        #     for i in range(0, len(L) - 1, 2) :
        #         vertex = (int(L[i]), int(L[i + 1]))
        #         polygon.append(vertex)
        #     # print('polygon',polygon)
        #     polygons.append(polygon)

        # print('centroid',x0,y0)
        # print('rotation',rotation)
        # print('perimeter',perimeter)

        posx, posy = zip(*perimeter) # convert list of 2 point to 2 tuples  of x and y
        posx = list(posx)
        posy = list(posy)
        # print('posx',posx)
        # rotate Figure
        for i, (x,y) in enumerate(zip(posx,posy)):
            posx[i], posy[i] = rotate((x0,y0), (x,y), rotation)

        # plt.plot(posx,posy)
        # plt.scatter(x0,y0)
        # plt.show()
        # perimeter = [(x,y) for x,y in zip(posx,posy)] # convert back to points

        output = open("data_files/input.txt", 'w')
        for x,y in zip(posx,posy):
            output.write(str(int(x)) + ' ' + str(int(y)) + ' ')

        for polygon in polygons:
            posx_polygon, posy_polygon = zip(*polygon)
            output.write('\n')
            posx_polygon = list(posx_polygon)
            posy_polygon = list(posy_polygon)
            for i, (x,y) in enumerate(zip(posx_polygon,posy_polygon)):
                posx_polygon[i], posy_polygon[i] = rotate((x0,y0), (x,y), rotation)
            for x,y in zip(posx_polygon,posy_polygon):
                output.write(str(int(x)) + ' ' + str(int(y)) + ' ')

        output.close()

        return (x0,y0)

    def plan(self, path):
        np.set_printoptions(precision=2)
        threshold_path = 6000 # threshold to do path only for big areas

        min_angle = 0
        max_angle = pi/2
        nb_angles = 4
        angles = np.linspace(min_angle, max_angle, nb_angles)

        last_total_cells_number = 1000000000000000

        #regenerating optimal map
        optimal_angle = -pi/2 # force angle
        if generate_input_file:
            center = input_file(path, rotation = optimal_angle)

        if generate_map :
            redraw(W, H)
            plt.close()

        if generate_decomposition :
            Boustrophedon_Cellular_Decomposition() #  generate the .bat decomposed_result file with cells

        decomposed, total_cells_number, cells = pickle.load(open("data_files/decomposed_result", "rb"))
        # print(('decomposed: {} total_cells_number: {} cells: {}').format(decomposed,total_cells_number,cells))
        print(('total_cells_number: {} with rotation: {}').format(total_cells_number,round(optimal_angle,2)))

        # removing cells that are too small
        remove_indexes = []
        for cell_id in range(1, total_cells_number + 1) :
            if cells[cell_id].points < threshold_path:
                remove_indexes.append(cell_id)

        total_cells_number = total_cells_number - len(remove_indexes)
        print(('Remove: {} cells based on min area threshold: {}').format(len(remove_indexes),threshold_path))
        cells = [i for j, i in enumerate(cells) if j not in remove_indexes]

        plt.ion()

        my_dpi = 100
        fig = plt.figure(figsize = (W / my_dpi, H / my_dpi), dpi = my_dpi, frameon = False)
        ax = plt.Axes(fig, [0., 0., 1., 1.])
        ax.set_axis_off()
        fig.add_axes(ax)

        # show decomposed result
        decomposed_image = np.zeros([H, W, 3], dtype = np.uint8)
        decomposed_image[decomposed > 0, :] = [255, 255, 255] # white
        plt.imshow(decomposed_image)
        ax.set_autoscale_on(False)

        display_cells(cells) # we should display only the biggest ones

        direction_set = [("LEFT", "DOWN"), ("LEFT", "UP"), ("RIGHT", "DOWN"), ("RIGHT", "UP")]
        # cities (start points, end points)
        points = np.zeros([4 * total_cells_number, 2, 2]) # 4 * total_cells_number city, start and end, x and y
        intra_path_length = np.zeros([4 * total_cells_number])
        for cell_id in range(1, total_cells_number + 1) :
            for i in range(4) :
                direction = direction_set[i]
                #print(cell_id,direction)
                #pprint(vars(cells[cell_id]))
                start_point, end_point, path_length, path = Boustrophedon_path(cells[cell_id], direction[0], direction[1], robot_radius)
                points[4 * (cell_id - 1) + i] = [start_point, end_point]
                # print(('points: {}').format(points))
                intra_path_length[4 * (cell_id - 1) + i] = path_length

        if generate_path_length :
            # generating distance matrix
            output_file = open("data_files/path_length.txt", "w")

            # intra path length
            for i in range(4 * total_cells_number) :
                print(round(intra_path_length[i], 8), end = ' ', file = output_file)
            print(file = output_file)

            # distance matrix
            distance_matrix = np.zeros([4 * total_cells_number, 4 * total_cells_number])
            for i in range(4 * total_cells_number) :
                for j in range(4 * total_cells_number) :
                    distance_matrix[i][j], path_transition = shortest_path(start = points[i][1], goal = points[j][0], plot = False, margin = robot_radius_shortest_path)
                    print(round(distance_matrix[i][j], 8), end = ' ', file = output_file)
                print(file = output_file)

            output_file.close()

        if generate_visiting_order :
            os.system("./optimal " + str(total_cells_number))

        optimal_output = open("data_files/optimal.txt")
        optimal_length = float(optimal_output.readline().rstrip())
        visiting_order = optimal_output.readline().rstrip()
        visiting_order = visiting_order.split(" -> ")

        path_total = []

        for i in range(total_cells_number) :
            cell_id = int(int(visiting_order[i]) / 4) + 1
            direction = direction_set[int(visiting_order[i]) % 4]

            start_point, end_point, path_length, path = Boustrophedon_path(cells[cell_id], direction[0], direction[1], robot_radius, plot = True, color = intra_path_color, width = path_width)
            figure()

            path_total.extend(path)

            cost, path_transition = shortest_path(start = points[int(visiting_order[i])][1], goal = points[int(visiting_order[(i + 1) % total_cells_number])][0], plot = True, color = inter_path_color, width = path_width, margin = robot_radius_shortest_path)
            path_total.extend(path_transition)

            if i == total_cells_number-1:
                plt.show(block=True)
            else:
                figure()

        path_contour = get_contour_path(path_total[0], plot = False, margin = robot_radius)
        path_total.extend(path_contour)

        if save_fig :
            plt.savefig("data_files/result" + ".jpg")

        output = open("data_files/fullpath.txt", 'w')
        posx, posy = zip(*path_total) # convert list of 2 point to 2 tuples  of x and y
        posx = list(posx)
        posy = list(posy)
        for x,y in zip(posx,posy):
            x_out, y_out = rotate(center, (x,y), -optimal_angle) # rotate back points
            output.write(str(int(x_out)) + ' ' + str(int(y_out)) + ' ')
        output.close()

	def callback_path(self, input_path):
		nb_pts_path = len(input_path.poses)
        rospy.loginfo("Perimeter of %i extremities to plan.", nb_pts_path)
        self.plan(input_path)

if __name__ == '__main__':
	try:
		path_planner_from_perimeter = path_planner_from_perimeter()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
