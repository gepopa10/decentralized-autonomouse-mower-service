import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import matplotlib.image as mpimg
from heapq import *
import pyclipper
from math import sqrt, factorial
import itertools
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from descartes import PolygonPatch

# color, width : inter path
def shortest_path(start, goal, plot = False, color = None, width = None, margin = 0) :

    def visible(i, j) :
        A, B = vertices[i], vertices[j]
        for polygon_edge in polygon_edges :
            if i != polygon_edge[0] and i != polygon_edge[1] and j != polygon_edge[0] and j != polygon_edge[1] :
                C, D = vertices[polygon_edge[0]], vertices[polygon_edge[1]]
                if intersection(A, B, C, D) :
                    return False
        return True

    def distance(A, B) :
        return np.sqrt(np.sum((B - A) ** 2))

    # check if C is on AB
    def on_segment(A, B, P) :
        Ax, Ay, Bx, By, Px, Py = A[0], A[1], B[0], B[1], P[0], P[1]
        if min(Ax, Bx) <= Px + 1E-6 and Px <= max(Ax, Bx) + 1E-6 and min(Ay, By) <= Py + 1E-6 and Py <= max(Ay, By) + 1E-6 :
            return True
        return False

    # AB CD
    def intersection(A, B, C, D) :
        Ax, Ay, Bx, By, Cx, Cy, Dx, Dy = A[0], A[1], B[0], B[1], C[0], C[1], D[0], D[1]
        # m1 = m2
        if (Ay - By) * (Cx - Dx) == (Cy - Dy) * (Ax - Bx) :
            return False
        # m1 = inf, x = Ax
        if Ax == Bx :
            m2 = (Cy - Dy) / (Cx - Dx)
            b2 = Cy - m2 * Cx
            P = (Ax, m2 * Ax + b2)
        # m2 = inf, x = Cx
        elif Cx == Dx :
            m1 = (Ay - By) / (Ax - Bx)
            b1 = Ay - m1 * Ax
            P = (Cx, m1 * Cx + b1)
        else :
            m1 = (Ay - By) / (Ax - Bx)
            b1 = Ay - m1 * Ax
            m2 = (Cy - Dy) / (Cx - Dx)
            b2 = Cy - m2 * Cx
            Px = (b1 - b2) / (m2 - m1)
            Py = m1 * Px + b1
            P = (Px, Py)
        if on_segment(A, B, P) and on_segment(C, D, P) :
            return True
        return False

    vertices = list()
    polygon = list() # the polygon for corresponding vertex
    polygon_edges = list() # edge with vertex represented in index
    vertex_num = 0
    polygon_id = 1
    input = open("data_files/input.txt", "r")
    vertices_for_nth_solution = list()
    for line in input :
        vertices_nth = list()
        L = (line.strip()).split(' ')
        cur_vertex_num = 0
        for i in range(0, len(L) - 1, 2) :
            vertex = (int(L[i]), int(L[i + 1])) # x, y
            vertices.append(vertex)
            vertices_nth.append(vertex)
            cur_vertex_num = cur_vertex_num + 1
            polygon.append(polygon_id)
        for i in range(cur_vertex_num - 1) :
            polygon_edges.append((vertex_num + i, vertex_num + i + 1))
        polygon_edges.append((vertex_num + cur_vertex_num - 1, vertex_num))
        vertex_num = vertex_num + cur_vertex_num
        polygon_id = polygon_id + 1

        vertices_for_nth_solution.append(vertices_nth)

    input.close()


    # print(polygon_edges) # [(0, 1), (1, 2), (2, 3)... these are the indexes
    # print(vertices) # [(309, 274), (300, 297), (289, 303) ... these are the actual coordinate
    # print(polygon) # polygon ids for each point, if only perimeter it should all belong to 1

    # posx, posy = zip(*vertices) # convert list of 2 point to 2 tuples  of x and y
    # plt.plot(list(posx), list(posy), label = 'polygon')
    # for i in range(len(posx)):
    #     plt.annotate(i, (posx[i],posy[i]))

    # plt.show()

    # print('vertices before',vertices, len(vertices))
    # margin = 10
    solutions = list()
    for n_sol in range(max(polygon)):
        pco = pyclipper.PyclipperOffset()
        pco.AddPath(vertices_for_nth_solution[n_sol], pyclipper.JT_SQUARE, pyclipper.ET_CLOSEDPOLYGON) # JT_MITER, JT_ROUND, JT_SQUARE
        if n_sol > 0:
            margin = -margin # we change the sign if we have obstacle because we want to increase boudary around
        solution = pco.Execute(-margin) # dont return the last point, we need to add the first at the end
        solutions.append(solution[0])
        # just for plotting
        for polygon in solution:
            # print(polygon)
            polygon.append(polygon[0])
            posx, posy = zip(*polygon) # convert list of 2 point to 2 tuples  of x and y
            if plot :
                plt.plot(list(posx), list(posy), label = 'polygon')
                # for i in range(len(posx)):
                #     plt.annotate(i, (posx[i],posy[i]))

    #plt.show()

    # print(solution[0])
    # vertices_margin = []
    # for point in solution[0]:
    #     vertices_margin.append((point[0],point[1]))
    # # vertices = [(solution[0][i][0],solution[0][i][1]) for i in zip(solution[0])] # convert back to points
    # vertices = vertices_margin
    # print('vertices after',vertices, len(vertices))

    vertices = list()
    polygon = list() # the polygon for corresponding vertex
    polygon_edges = list() # edge with vertex represented in index
    vertex_num = 0
    polygon_id = 1

    for polygon_sol in solutions :
        cur_vertex_num = 0
        for i in range(0, len(polygon_sol)) :
            vertex = (int(polygon_sol[i][0]), int(polygon_sol[i][1])) # x, y
            vertices.append(vertex)
            cur_vertex_num = cur_vertex_num + 1
            polygon.append(polygon_id)
        for i in range(cur_vertex_num - 1) :
            polygon_edges.append((vertex_num + i, vertex_num + i + 1))
        polygon_edges.append((vertex_num + cur_vertex_num - 1, vertex_num))
        vertex_num = vertex_num + cur_vertex_num
        polygon_id = polygon_id + 1

    # print('vertices after',vertices, len(vertices))

    vertices.append(start)
    polygon.append('S')
    vertices.append(goal)
    polygon.append('G')
    start_index, goal_index = vertex_num, vertex_num + 1
    vertex_num = vertex_num + 2
    vertices = np.asarray(vertices)
    connected = np.zeros([vertex_num, vertex_num], dtype = bool)

    for polygon_edge in polygon_edges :
        connected[polygon_edge[0]][polygon_edge[1]] = True
        connected[polygon_edge[1]][polygon_edge[0]] = True

    for i in range(vertex_num) :
        for j in range(i + 1, vertex_num) :
            # not connected : not polygon edge, check connection
            if (not connected[i][j]) and (polygon[i] != polygon[j]) and visible(i, j): #set connected between two points if visible
                # print('here')
                connected[i][j] = True
                connected[j][i] = True

    # print(np.shape(connected)) # nbpointsxnbpoints
    # Dijkstra
    Q = [] # cost, parent, vertex
    # print(vertex_num) # nb of points
    parent = np.full((vertex_num), -1, dtype = int)
    cost = np.full((vertex_num), np.inf, dtype = float)
    heappush(Q, (0, start_index, start_index))
    while Q and parent[goal_index] == -1 :
        (current_cost, p, u) = heappop(Q)
        if parent[u] == -1 :
            parent[u] = p
            cost[u] = current_cost
            for v in range(vertex_num) :
                if connected[u][v] and parent[v] == -1 :
                    # print(vertices[u], vertices[v])
                    heappush(Q, (current_cost + distance(vertices[u], vertices[v]), u, v))

    path_points = []

    cur_index = goal_index
    while cur_index != start_index :
        u, v = parent[cur_index], cur_index

        start_x = vertices[u][0]
        start_y = vertices[u][1]
        end_x = vertices[v][0]
        end_y = vertices[v][1]

        path_points.append((start_x,start_y))
        path_points.append((end_x,end_y))

        cur_index = parent[cur_index]

    # print('path_points',path_points)
    # path_points = list(path_points for path_points,_ in itertools.groupby(path_points)) # remove same points
    new_path_points = []
    for elem in path_points: # remove same points
        if elem not in new_path_points:
            new_path_points.append(elem)
    path_points = new_path_points

    # print('end_x',start_x,'end_y',start_y)
    # -------------------------------------------------------------
    # THIS IS NOT ENOUGH, WE MIGHT HAVE A POINT NEAR BUT THROUGH AN OBSTACLE
    path_points.sort(key = lambda p: sqrt((p[0] - start_x)**2 + (p[1] - start_y)**2)) # sort by proximity to start

    # Remove points that are very close
    close_threshold = 5 # how close two points can be to be removed
    to_remove_index = list()
    for i in range(len(path_points)-1):
        if Point(path_points[i]).distance(Point(path_points[i+1])) < close_threshold:
            to_remove_index.append(i)

    for idx in to_remove_index[::-1]: # reverse order to not thrown subsequent indexes and change place of items to remove
        path_points.pop(idx)

    # MAKE SURE THAT WE DONT PASS THROUGH AN OBSTACLE
    epsilon = 1 # threshold if a point is in polygon or not
    nb_pts_line = 100 # nb of points on a line between two points to see if the line passes through polygon

    nb_lines = len(path_points) - 1 # number of pairs in other words

    path_points_permuted = [path_points.pop(0)] # we start with the first point

    out_polygon_sol= solutions.pop(0) # we get the polygon points of the perimeter

    points = list()
    for point in out_polygon_sol:
        points.append((point[0], point[1]))
    polygon_points = Polygon(points)

    polygon_points_obstacles = list()
    for polygon_sol in solutions: # we get the rest of the points of obstacles
        points = list()
        for point in polygon_sol:
            points.append((point[0], point[1]))
        polygon_points_obstacles.append(Polygon(points))

    for line in range(nb_lines): # could be a while len(path_points_permuted)!=len(path_points)-1

        scores = [] #stores the scores of all combinations

        for i in range(len(path_points)): # we loop on the different possible next points
            # Create a discretized line to check if any point is within obstacle
            line_points = list(get_equidistant_points(path_points_permuted[-1], path_points[i], nb_pts_line))

            score = [] #score for a line
            for point in line_points:
                in_polygon =  (Point(point).distance(polygon_points) < epsilon) # check if point is inside polygon, should be inside permimeter polygon and outside the others, better to use epsilon if very small number
                score.append(in_polygon)

                # THIS IS TO CONSIDER OBSTACLES BUT DOESNT WORK!
                # for polygon_points_obstacle in polygon_points_obstacles:
                #     out_polygon = (Point(point).distance(polygon_points_obstacle) > epsilon)
                #     score.append(out_polygon)

            scores.append(sum(score)) # we gather all the scores and to choose the max!
            if all(score): # we are good because the closest points are first so we stop if we have all points in polygon
                break

        #find the index of lowest score
        index_best_combination = scores.index(max(scores)) # the one that has maximum pts in polygon is the next best combination!
        path_points_permuted.append(path_points.pop(index_best_combination))

    # Plot patches for debug
    # polygon_patch = PolygonPatch(polygon_points)
    # ax.add_patch(polygon_patch)
    # for polygon_points_obstacle in polygon_points_obstacles:
    #     polygon_patch = PolygonPatch(polygon_points_obstacle, color='black')
    #     ax.add_patch(polygon_patch)

    path_points = path_points_permuted

    path = []
    # create discretize path
    for i in range(len(path_points)-1):
        start_x = path_points[i][0]
        start_y = path_points[i][1]
        end_x = path_points[i+1][0]
        end_y = path_points[i+1][1]

        dist = sqrt((end_x-start_x)**2 + (end_y-start_y)**2)
        nb_points = int(dist) - 1 # we want a point at each pixel

        x_path = np.linspace(start_x, end_x, nb_points).tolist()
        y_path = np.linspace(start_y, end_y, nb_points).tolist()
        path_points_discretize = []

        if i == len(path_points) -1:
            rn = len(x_path)
        else:
            rn =len(x_path)-1 # we dont want to put the last point except at the end of path

        for i in range(rn):
            path_points_discretize.append((x_path[i], y_path[i]))

        path_points_discretize = list(zip(x_path, y_path))
        # print('path_points',path_points)
        # print('x_path',x_path)
        # print('y_path',y_path)
        path.extend(path_points_discretize)


    if plot :
        # idx = 0
        # cur_index = goal_index
        # while cur_index != start_index :
        #     u, v = parent[cur_index], cur_index
        #
        #     start_x = vertices[u][0]
        #     start_y = vertices[u][1]
        #     end_x = vertices[v][0]
        #     end_y = vertices[v][1]
            # dist = sqrt((end_x-start_x)**2 + (end_y-start_y)**2)
            # nb_points = int(dist) - 1 # we want a point at each pixel

            # x_path = np.linspace(start_x, end_x, nb_points).tolist()
            # y_path = np.linspace(start_y, end_y, nb_points).tolist()
            # path_points = []
            # for i in range(len(x_path)):
            #     path_points.append((x_path[i],y_path[i]))

            # path_points = list(zip(x_path, y_path))
            # print('path_points',path_points)
            # print('x_path',x_path)
            # print('y_path',y_path)
            # path.extend(path_points)

            # plt.plot([vertices[u][0], vertices[v][0]], [vertices[u][1], vertices[v][1]], color = color, linewidth = width, zorder = 4)

            # plt.annotate(idx, (start_x,start_y))
            # idx = idx + 1
            # plt.annotate(idx, (end_x,end_y))
            # idx = idx + 1
            # # print([vertices[u][0], vertices[v][0]], [vertices[u][1], vertices[v][1]])
            # print(start_x, start_y, end_x, end_y)
            # # plt.scatter([vertices[u][0], vertices[v][0]],[vertices[u][1], vertices[v][1]], color = 'r')
            # # plt.scatter(x_path,y_path, color = 'r')
            #
            # posx, posy = zip(*path) # convert list of 2 point to 2 tuples  of x and y
            # # plt.scatter(posx,posy)
            # cur_index = u
            # plt.show()

        posx, posy = zip(*path) # convert list of 2 point to 2 tuples  of x and y
        # print('path',path)
        # print('posx',posx)
        # print('posy',posy)
        plt.plot(posx, posy, color = color, linewidth = width, zorder = 4)
        # for i in range(len(posx)):
        #     plt.annotate(i, (posx[i],posy[i]))
        plt.annotate(-1, (posx[-1],posy[-1]))
        plt.scatter(posx[0], posy[0], color = 'r')
        plt.show()

    return cost[goal_index], path

def get_equidistant_points(p1, p2, parts):
    return zip(np.linspace(p1[0], p2[0], parts+1),
               np.linspace(p1[1], p2[1], parts+1))

if __name__ == "__main__" :
    start, goal = (201, 742), (1025, 1435)
    start, goal = (12, 1704), (704, 1546)
    color = 'dimgray'
    width = 3.2
    #[(222.0, 1468.0), (651.0, 1470.0), (704.0, 1546.0)]
    #[(12.0, 1704.0), (220.0, 1470.0), (222.0, 1468.0), (649.0, 1468.0), (651.0, 1470.0), (704.0, 1546.0)]
    image = mpimg.imread("data_files/map.jpg")
    H, W = image.shape[0], image.shape[1]
    # print(H,W)
    my_dpi = 100
    fig = plt.figure(figsize = (W / my_dpi, H / my_dpi), dpi = my_dpi, frameon = False)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    ax.imshow(image, aspect = "auto")

    plt.plot(start[0], start[1], marker = 'o', color = 'orange', markersize = width + 6.4, zorder = 5)
    plt.plot(goal[0], goal[1], marker = 'X', color = 'springgreen', markersize = width + 6.4, zorder = 5)

    cost, path = shortest_path(start = start, goal = goal, plot = True, color = color, width = width, margin = 5)
    print(cost)
