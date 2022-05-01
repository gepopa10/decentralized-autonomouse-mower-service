#!/usr/bin/env python

import time
from math import pi
import scipy.signal
import numpy as np
import os
import matplotlib.pyplot as plt

if __name__ == '__main__':

    # tips:
    # use rosrun plotjuggler PlotJuggler to vizualize .bag file
    # extract topic from .bag and put it into csv with:
    # rostopic echo -b $bag_file_name.bag -p $topic_name > $desired_file_name.csv

    this_folder = os.path.dirname(os.path.abspath(__file__))
    datafile = 'encoder_data/raw_encoder_data.csv' # this was extracted from the .bag
    data_path = '{}/{}'.format(this_folder,datafile)

    data = np.loadtxt(data_path, delimiter=',', skiprows=1) # we skip first row because its the header
    t = [x - data[0,0] for x in data[:,0]]
    dts = [y - x for x,y in zip(t,t[1:])] # this give weird results so we force dt = 0.01
    dt = sum(dts) / len(dts)
    dt = 0.01
    raw_encoder_data = data[:,1]

    print('Number of points: ',len(raw_encoder_data))
    print('dt: ',dt)

    # variable for the savgol filter to test
    window_sizes = [5,11,23]
    polynomial_order_filters = [0,1,2]
    total_optimization_trial = len(window_sizes)*len(polynomial_order_filters)
    fig, ax = plt.subplots(len(window_sizes),len(polynomial_order_filters)+1) # +1 for kalman
    ite = 0

    for i in range(len(window_sizes)):
        for j in range(len(polynomial_order_filters)):
            ite += 1
            window_size = window_sizes[i] # window size for filtering, must be odd
            polynomial_order_filter = polynomial_order_filters[j] # polynomial order for filtering, works best with 1
            print('---------------------------------')
            print('Ite ',ite,'/',total_optimization_trial,' -> Window size: ',window_size,' polynomial order: ',polynomial_order_filter)
            speeds_for_filtering = []

            speeds_filtered = [] # here we store all the values filtered
            idx = 0

            while True:
                speeds_for_filtering.append(raw_encoder_data[idx])
                speed_filtered = raw_encoder_data[idx]
                idx += 1
                #print('len(speeds_for_filtering)',len(speeds_for_filtering))
                if (len(speeds_for_filtering) >= window_size):
                    speeds_filtered_from_window = scipy.signal.savgol_filter(speeds_for_filtering, window_size, polynomial_order_filter)
                    speed_filtered = speeds_filtered_from_window[-1]
                    speeds_for_filtering = speeds_for_filtering[1::] #remove the first value

                speeds_filtered.append(speed_filtered) # might be raw until we reach window size

                if idx == len(raw_encoder_data):
                    break

            ax[i,j].plot(t, speeds_filtered, label='savgol window size: %i, order: %i' %(window_size,polynomial_order_filter))
            ax[i,j].scatter(t, raw_encoder_data, label='raw', c='k',  s=10)
            # plt.legend(loc='best')
            ax[i,j].set_title('savgol window size: %i, order: %i' %(window_size,polynomial_order_filter))

    from linear_EKF import EKF_DIST
    P_EKF_Dist = np.eye(3)
    x_hat_EKF_Dist = np.zeros([3,1])
    speeds_filtered = []
    idx = 0
    while True:

        x_hat_EKF_Dist,P_EKF_Dist = EKF_DIST(raw_encoder_data[idx],x_hat_EKF_Dist,P_EKF_Dist,dt)
        idx += 1
        speeds_filtered.append(x_hat_EKF_Dist[0][0])
        if idx == len(raw_encoder_data):
            break

    ax[0,len(polynomial_order_filters)].plot(t, speeds_filtered, label='kalman')
    ax[0,len(polynomial_order_filters)].scatter(t, raw_encoder_data, label='raw', c='k',  s=10)
    ax[0,len(polynomial_order_filters)].set_title('kalman')
    # plt.legend(loc='best')
    plt.show()
