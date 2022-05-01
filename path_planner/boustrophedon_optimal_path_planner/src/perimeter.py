import numpy as np
from math import sin, cos, atan, pi, sqrt
import matplotlib.pyplot as plt
import ast

def fit_contour(path, datafile, plot = False):

    input = open(path+datafile, "r")

    # for csv geolocation debug
    # position_prediction = list()
    # for line in input :
    #     # print('line',char(line))
    #     # as_list = ast.literal_eval(line)
    #     # print(ast.literal_eval(as_list)[0])
    #     position_prediction.append(ast.literal_eval(ast.literal_eval(line)))
    #     # print('ast.literal_eval(line)',ast.literal_eval(line))
    #
    # # print(position_prediction[:][0:2])
    # posx=[row[0] for row in position_prediction]
    # posy=[row[1] for row in position_prediction]

    # # for csv Vrep
    # posx, posy = np.loadtxt(fullPath, delimiter=',', unpack=True)

    # for csv ugv_control_interface_v2
    data = np.loadtxt(fullPath, delimiter=',')
    t = data[:,0]
    # posx = list(data[:,1]) #raw
    # posy = list(data[:,2])
    posx = list(data[:,8]) #UKF
    posy = list(data[:,9])

    # posx = posx[100::] #remove first points
    # posy = posy[100::]
    posx = posx[:-900] #remove last points
    posy = posy[:-900]
    # posx = posx[0::2] # remove half of the points
    # posy = posy[0::2]


    #rotate lists, # play with this to make the contour work, starting at the top works best
    # len_list = len(posx)
    # nb_rotations = -300
    # posx = rotate(posx,nb_rotations)
    # posy = rotate(posy,nb_rotations)


    posx_raw, posy_raw = posx, posy

    if plot:
        fig, ax = plt.subplots()
        ax.scatter(posx, posy, label='Raw', c='k',  s=10)
        # Start/Goal
        ax.scatter(posx[0],posy[0], s=60, label='Start', c='g')
        ax.scatter(posx[-1],posy[-1], s=60, label='Goal', c='r')

    posx, posy = filter_distance(posx,posy)
    x0, y0 = np.mean(posx).tolist(), np.mean(posy).tolist() # important that we find a centroid within the perimeter!

    if plot:
        plt.scatter(x0, y0, label='Centroid', color='magenta')
        # ax.scatter(posx, posy, label='Distance filter', c='r', lw=1)

    fit_points_x, fit_points_y = contour_anticlockwise(posx, posy, x0, y0)

    if plot:
        plt.plot(fit_points_x, fit_points_y, label='fit', color='b')
        plt.legend(loc='best')
        # plt.show()

    return fit_points_x, fit_points_y, x0, y0, posx_raw, posy_raw

def rotate(l, n):
    return l[n:] + l[:n]

def filter_distance(posx,posy):
    # filter data to eliminate data points next to each other but too far
    # based on threshold_norm
    threshold_norm = 1.3 #130cm
    idx = 0

    while posx[idx]!=posx[-2]: # stop when we are at then end of the list -1
        norm_next_point = sqrt((posx[idx+1]-posx[idx])**2 + (posy[idx+1]-posy[idx])**2)
        if norm_next_point > threshold_norm or posx[idx+1] == posx[idx] or posy[idx+1] == posy[idx]:
            posx.pop(idx+1)
            posy.pop(idx+1)
        else:
            idx = idx + 1

    return posx, posy

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def save_results(path, datafile, posx, posy, x0, y0, range_meters = 25, dpi = 1000) :
    # write to text file
    fullPath = path+datafile
    file = open(fullPath,'w')

    fitted_x = posx
    fitted_y = posy
    # we add the centroid at the beggining
    fitted_x.insert(0, x0)
    fitted_y.insert(0, y0)

    min_fitted = -range_meters
    max_fitted = range_meters

    dpi = 1000
    nb_pts = 0
    for x,y in zip(fitted_x,fitted_y):
        nb_pts = nb_pts + 1
        #normalize data between 0-1
        x = 1.0/( max_fitted - min_fitted)*(x - max_fitted) + 1
        y = 1.0/( max_fitted - min_fitted)*(y - max_fitted) + 1
        file.write(str(int(x*dpi)) + ' ' + str(int(y*dpi)) + ' ')

    print(('Written {} points to {} file.').format(nb_pts,fullPath))
    file.close()

def read_results(path, datafile, datafile_out, range_meters = 25, dpi = 1000, translation = {'x':0, 'y':0}):
    min_fitted = 0 #why it was -range_meters?
    max_fitted = range_meters

    # get results
    input = open(path+datafile,'r')

    fullpath = list()
    firstline = input.readline()
    L = (firstline.strip()).split(' ')

    for i in range(0, len(L), 2) :
        x_norm = int(L[i])/dpi
        y_norm = int(L[i + 1])/dpi
        # denormalize
        x = (x_norm - 1)*(max_fitted - min_fitted) + max_fitted - translation['x'] # we remove the translation we added before
        y = (y_norm - 1)*(max_fitted - min_fitted) + max_fitted - translation['y']
        point = (x, y)
        fullpath.append(point)

    posx, posy = zip(*fullpath) # convert list of 2 point to 2 tuples  of x and y
    # assert posx[0] != posx[-1]
    # assert posy[0] != posy[-1]
    plt.plot(posx, posy)
    plt.scatter(posx[0], posy[0], label='Start-End path')
    plt.legend(loc='best')
    plt.show()

    # get results
    file = open(path+datafile_out,'w')

    nb_pts = 0
    for x,y in zip(posx,posy):
        nb_pts = nb_pts + 1
        file.write(str(x) + ' ' + str(y) + ' ')

    print(('Written {} points to {} file.').format(nb_pts,datafile_out))
    file.close()

    return posx, posy

def to_csv_file(path, datafile, posx, posy):
    import csv

    rows = zip(posx, posy)
    with open(path + datafile, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        # header = ['self.t_list', 'self.xdes_list', 'self.x_list', 'self.zdes_list', 'self.z_list', \
        #         'self.ydes_list', 'self.y_list', 'self.dxdes_list', 'self.dx_list', 'self.dzdes_list', \
        #         'self.dz_list', 'self.dydes_list', 'self.dy_list', 'self.phi_list', 'self.theta_list', 'self.psi_list']
        # writer.writerow(header)
        for row in rows:
            writer.writerow(row)

def animate(posx, posy, posx_raw, posy_raw):

    print('duration: {} ms.'.format(len(posx)))

    from matplotlib import animation
    from matplotlib.patches import Circle
    # First set up the figure, the axis, and the plot element we want to animate
    fig = plt.figure()
    ax = plt.axes(xlim=(min(posx_raw), max(posx_raw)), ylim=(min(posy_raw), max(posy_raw)))
    ax.scatter(posx_raw, posy_raw, label='Raw', c='k', lw=1)
    line, = ax.plot([], [], lw=21, color = 'green', label='Path')
    radius_robot = 0.30 # in meters
    interval_ms = 1

    # n = 100
    # posx_2D = [posx[i:i+n] for i in range(0, len(posx), n)]
    # posy_2D = [posy[i:i+n] for i in range(0, len(posy), n)]

    # take only every 20 points
    # posx = posx[::20]
    # posy = posy[::20]
    # interval_ms = 30

    def func_animate(i):
        line.set_data(posx[:i], posy[:i])
        # line.set_data(posx_2D[i], posy_2D[i])
        # c1 = Circle((posx[i], posy[i]), radius_robot, color='green', alpha = 0.5)
        # ax.add_artist(c1)
        print('animation step: {}/{}'.format(i,len(posx)))
        return line,

    def init():
        line.set_data([], [])
        return line,

    anim = animation.FuncAnimation(fig, func_animate, init_func=init,
                                   frames=len(posx), interval=interval_ms, blit=True)

    anim.save(r'animation.gif', fps=60)
    # anim.save('animation.mp4', fps=60, extra_args=['-vcodec', 'libx264'])
    plt.show()

def contour_anticlockwise(posx, posy, x0, y0):
    vector_x = []
    vector_y = []
    angle = []
    search_radius = 0.5 # better to be as small as possible, but if too small it can get stuck for real yard 1m was required
    search_radius_start = 0.5
    fit_points_x = [posx[0], posx[1]]
    fit_points_y = [posy[0], posy[1]]

    ###########################################################################
    # select the point start at the edge
    ###########################################################################

    last_dist_centroid = sqrt((x0 - fit_points_x[-1])**2 + (y0 - fit_points_y[-1])**2)
    idx = 0
    fit_points_x = [posx[0]]
    fit_points_y = [posy[0]]
    for x,y in zip(posx[1:],posy[1:]):
        idx = idx + 1
        norm = sqrt((fit_points_x[-1] - x)**2 + (fit_points_y[-1] - y)**2)
        if norm < 0.5:
            dist_centroid = sqrt((x0 - x)**2 + (y0 - y)**2)
            if dist_centroid < last_dist_centroid:
                last_dist_centroid = dist_centroid
                idx_1stpoint = idx


    fit_points_x = [posx[idx_1stpoint]]
    fit_points_y = [posy[idx_1stpoint]]

    ###########################################################################
    # select the point with min angle to centroid and less than 90 deg diff from last vector
    ###########################################################################

    search_points_x = []
    search_points_y = []
    for x,y in zip(posx,posy):
        # if x not in fit_points_x and y not in fit_points_y: #avoid finding the same old points
        norm = sqrt((fit_points_x[-1] - x)**2 + (fit_points_y[-1] - y)**2)
        if norm < search_radius_start:
            search_points_x.append(x)
            search_points_y.append(y)

        if len(search_points_x)==0:
            print('didnt find any search points, breaking...')
            break

    next_x = search_points_x[0]
    next_y = search_points_y[0]
    # select the point with min angle to centroid and less than 90 deg diff from last vector
    last_diff_angle_centroid = 2*pi #everyone is better than 2pi

    for x,y in zip(search_points_x,search_points_y):
        angle = np.angle((x - fit_points_x[-1]) + 1j * (y - fit_points_y[-1]))
        if angle < 0:
            angle = angle + 2*pi

        angle_centroid = np.angle((x0 - fit_points_x[-1]) + 1j * (y0 - fit_points_y[-1]))
        if angle_centroid < 0:
            angle_centroid = angle_centroid + 2*pi

        diff_angle_centroid = angle_centroid - angle
        if diff_angle_centroid < 0:
            diff_angle_centroid = diff_angle_centroid + 2*pi

        condition = diff_angle_centroid < last_diff_angle_centroid
        if condition:
            next_x = x
            next_y = y
            last_diff_angle_centroid = diff_angle_centroid

    fit_points_x.append(next_x)
    fit_points_y.append(next_y)

    angle_centroid = np.angle((x0 - fit_points_x[-1]) + 1j * (y0 - fit_points_y[-1]))
    if angle_centroid < 0:
        angle_centroid = angle_centroid + 2*pi

    angle_prev = angle_centroid # we fake it to start anticlockwise
    angle = np.angle((fit_points_x[-1] - fit_points_x[-2]) + 1j * (fit_points_y[-1] - fit_points_y[-2]))
    if angle < 0:
        angle = angle + 2*pi
    print('angle_prev',angle_prev,'angle first line',angle)
    angle_prev = angle # better like this?

    # shows first points selected to start algorithm
    # for i in range(len(fit_points_x)):
    #     plt.annotate(i, (fit_points_x[i], fit_points_y[i]))
    # plt.scatter(fit_points_x, fit_points_y, label='fit', color='b')
    # plt.show()

    ###########################################################################
    ###########################################################################
    ###########################################################################

    angles = [] # store angles of vectors to rotate the points after
    notend_perimeter = True
    iteration = 0
    stop_iteration = 500 # max iteration, usefull for debug also
    while notend_perimeter:
        iteration = iteration + 1

        # create search points from last point based on search_radius
        search_points_x = []
        search_points_y = []
        for x,y in zip(posx,posy):
            # if x not in fit_points_x and y not in fit_points_y: #avoid finding the same old points
            norm = sqrt((fit_points_x[-1] - x)**2 + (fit_points_y[-1] - y)**2)
            if norm < search_radius:
                search_points_x.append(x)
                search_points_y.append(y)

        if len(search_points_x)==0:
            print('didnt find any search points, breaking...')
            break

        next_x = search_points_x[0]
        next_y = search_points_y[0]
        # print(search_points_x)
        # print(fit_points_x)
        if iteration == stop_iteration:
            # plt.scatter(search_points_x, search_points_y, label='search_points %i' %iteration)
            # plt.annotate('Start', (fit_points_x[-1], fit_points_y[-1]), xytext=(fit_points_x[-1]-0.15, fit_points_y[-1]))
            # plt.scatter(fit_points_x[-1], fit_points_y[-1], label='start %i' %iteration, alpha=0.5)
            pass


        # select the point with min angle to centroid and less than 90 deg diff from last vector
        last_diff_angle_centroid = 2*pi #everyone is better than 2pi
        angle_last = 0
        norm_last = sqrt((fit_points_x[-1] - next_x)**2 + (fit_points_y[-1] - next_y)**2)

        idx_angle = 0
        idx_angle_final = 0
        for x,y in zip(search_points_x,search_points_y):
            # if x not in fit_points_x and y not in fit_points_y: #avoid finding the same old points
            idx_angle = idx_angle + 1
            # angle = atan((y-fit_points_y[-1])/(x-fit_points_x[-1])) # this point to the last point
            # angle_centroid = atan((y0-y)/(x0-x)) # this point to the centroid
            # print('x',x,'fit_points_x[-1]',fit_points_x[-1],'y',y,'fit_points_y[-1]',fit_points_y[-1])
            angle = np.angle((x - fit_points_x[-1]) + 1j * (y - fit_points_y[-1]))
            if angle < 0:
                angle = angle + 2*pi
            # print('x',x,'x0',x0,'y',y,'y0',y0)
            angle_centroid = np.angle((x0 - fit_points_x[-1]) + 1j * (y0 - fit_points_y[-1]))
            # angle_centroid = np.angle((fit_points_x[-1] - x0) + 1j * (fit_points_y[-1] - y0))
            if angle_centroid < 0:
                angle_centroid = angle_centroid + 2*pi
            # diff_angle_centroid = abs(angle_centroid - angle)
            diff_angle_centroid = angle_centroid - angle
            if diff_angle_centroid < 0:
                diff_angle_centroid = diff_angle_centroid + 2*pi
            # diff_angle_centroid = angle_between((x0 - fit_points_x[-1],y0 - fit_points_y[-1]),(x - fit_points_x[-1],y - fit_points_y[-1]))
            # diff_angle_prev = abs(angle_prev - angle)
            diff_angle_prev = angle_between((fit_points_x[-1]-fit_points_x[-2],fit_points_y[-1]-fit_points_y[-2]),(x-fit_points_x[-1],y-fit_points_y[-1]))
            if iteration == stop_iteration:
                pass
                # plt.annotate('%i' %idx_angle, (x, y), xytext=(x-0.05, y))
                # print('idx_angle',idx_angle,'angle',angle, 'angle_centroid', angle_centroid,'diff_angle_centroid', diff_angle_centroid,'diff_angle_prev',diff_angle_prev)
                # plt.annotate('%i' %idx_angle, (x, y), xytext=(x-0.05, y))
                # print('idx_angle',idx_angle,'angle',angle,'angle_last',angle_last,'diff_angle_prev',diff_angle_prev)
                # print('points',fit_points_x[-2],fit_points_y[-2],fit_points_x[-1],fit_points_y[-1],x,y)


            # condition = angle > angle_last
            condition = diff_angle_centroid < last_diff_angle_centroid
            if diff_angle_prev < pi/3 and condition:
                norm = sqrt((fit_points_x[-1] - x)**2 + (fit_points_y[-1] - y)**2)
                # print('here diff_angle_centroid', diff_angle_centroid, 'last_diff_angle_centroid', last_diff_angle_centroid)
                # if norm < norm_last: #take the smallest steps
                #     next_x = x
                #     next_y = y
                #     angle_last = angle
                #     norm_last = norm
                    # last_diff_angle_centroid = diff_angle_centroid
                next_x = x
                next_y = y
                angle_last = angle
                last_diff_angle_centroid = diff_angle_centroid
                idx_angle_final = idx_angle

        # print('iteration',iteration,'chose: ', next_x, next_y)
        # print('angle_prev', angle_prev, 'angle_last', angle_last, 'idx_angle_final', idx_angle_final)
        if angle_last == 0: # we didnt find a good point
            idx_not_good_x = min(range(len(posx)), key=lambda i: abs(posx[i]-fit_points_x[-1]))
            idx_not_good_y = min(range(len(posy)), key=lambda i: abs(posy[i]-fit_points_y[-1]))
            idx_not_good = min(range(len(posx)), key=lambda i: abs(sqrt(((posx[i] - fit_points_x[-1])**2)+(posy[i] - fit_points_y[-1])**2)))
            # print(idx_not_good_x,idx_not_good_y)
            print('Delete', (posx[idx_not_good], posy[idx_not_good]))

            # plt.annotate('Delete', (posx[idx_not_good], posy[idx_not_good]), xytext=(posx[idx_not_good]+0.05, posy[idx_not_good]))
            del posx[idx_not_good]
            del posy[idx_not_good]

            plt.scatter(fit_points_x[-1], fit_points_y[-1], color = 'r')
            if len(fit_points_x) > 2:
                fit_points_x.pop()
                fit_points_y.pop()
            # angle_prev = np.angle((fit_points_x[-1] - fit_points_x[-2]) + 1j * (fit_points_y[-1] - fit_points_y[-2]))
            # if angle_prev < 0:
            #     angle_prev = angle_prev + 2*pi
        else:
            angle_prev = angle_last
            fit_points_x.append(next_x)
            fit_points_y.append(next_y)
            angles.append(angle_prev)

        plt.annotate(iteration, (next_x, next_y))

        # end criteria -> same points reselected that are the same as the one at the beginning in fit_points
        if fit_points_x[0] == fit_points_x[-1] and fit_points_y[0] == fit_points_y[-1]:
            print('same points reselected that are the same as the one at the beginning in fit_points')
            notend_perimeter = False

        # end criteria -> dist of current point less than threshold to first point
        norm_to_first_point = sqrt((fit_points_x[-1] - fit_points_x[0])**2 + (fit_points_y[-1] - fit_points_y[0])**2)
        if norm_to_first_point < 0.5:
            print(('we finished contour arriving at the beggining within a proximity of {} m').format(round(norm_to_first_point,3)))
            notend_perimeter = False

        if iteration == stop_iteration:
            # print(fit_points_x)
            # print(fit_points_y)
            # plt.axis([-17, 7, -5, 19])
            plt.axis('square') # to view without deformation (real points absolute values)
            # plt.scatter(search_points_x, search_points_y, label='search_points %i' %iteration)
            # plt.scatter(fit_points_x[-1], fit_points_y[-1], label='last')
            # plt.scatter(fit_points_x[-2], fit_points_y[-2], label='last-1')
            plt.plot(fit_points_x, fit_points_y, label='fit', color='b')
            plt.legend(loc='best')
            plt.show()
            notend_perimeter = False

    print(('End in {} iterations.').format(iteration))

    return fit_points_x, fit_points_y

if __name__ == '__main__':

    path="data_files/"
    datafile = 'data20201029-150634.csv'
    # datafile = 'Debug_cour.csv'
    # datafile = 'Debug_chambre.csv'
    fullPath=path+datafile

    # fitted_x, fitted_y, x0, y0, posx_raw, posy_raw = fit_contour(path, datafile, plot = True)

    range_meters = 20
    dpi = 2000 #so 1m = 100dpi
    translation = {'x':0, 'y':11.1} #applied translation for the optimal path planner that takes only positive values
    # save_results(path, 'input_raw_29oct15h06.txt', fitted_x, fitted_y, x0, y0, range_meters, dpi) #we put this in case 1 as input_raw.txt input.txt is the rotated one
    posx, posy = read_results(path, 'fullpath.txt', 'fullpath_meters.txt', range_meters, dpi, translation)
    # to_csv_file(path, 'fullpath_29oct15h06.csv', posx, posy)
    # animate(posx, posy, posx_raw, posy_raw)
