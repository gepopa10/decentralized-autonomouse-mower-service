import numpy as np
from math import sin, cos, atan, pi, sqrt
import matplotlib.pyplot as plt

def find_centroid(x, y):
    assert (len(x)==len(y)), f'[ERROR] Length of x is {len(x)} while length of y is {len(y)} but they need to be the same!'

    n = len(x) # nb of points
    A = 0 # area of the polygon
    for i in range(n-1):
        A += 0.5*(x[i]*y[i+1]-x[i+1]*y[i])

    Cx = 0
    Cy = 0
    for i in range(n-1):
        Cx += 1/(6*A)*(x[i]+x[i+1])*(x[i]*y[i+1]-x[i]*y[i])
        Cy += 1/(6*A)*(y[i]+y[i+1])*(x[i]*y[i+1]-x[i]*y[i])

    return Cx, Cy, A

def transform_for_processing(x, y, x0, y0, dpi = 1000, translation = {'x':0, 'y':0}):
    assert (len(x)==len(y)), f'[ERROR] Length of x is {len(x)} while length of y is {len(y)} but they need to be the same!'

    translation['x']
    n = len(x) # nb of points
    x_out = [None]*n
    y_out = [None]*n

    for i in range(n):
        x_out[i] = int((x[i] + translation['x']) * dpi/10)
        y_out[i] = int((y[i] + translation['y']) * dpi/10)

    x0_out = int((x0 + translation['x']) * dpi/10)
    y0_out = int((y0 + translation['y']) * dpi/10)

    assert any(xi < 0 for xi in x_out) == False, '[ERROR] All numbers in x are not positive'
    assert any(yi < 0 for yi in y_out) == False, '[ERROR] All numbers in x are not positive'

    return x_out, y_out, x0_out, y0_out

if __name__ == '__main__':

    # Perimeter
    x = [2.33, 15.14, 15.14, 3.12, 3.12, 4.78, 4.78, 3.92, 3.92, 8.25, 9.57, 9.57, 10.82, 13.18, 14.42, 14.42, 13.18, 11.39, 9.49, 3.92, 3.92, 4.78, 4.78, 2.43]
    y = [-10.8, -10.8, 4.94, 4.94, 4.05, 4.05, 0.12, 0.12, -0.82, -0.82, 0.32, 2.08, 3.33, 3.33, 2.08, -0.32, -1.52, -1.53, -3.23, -3.08, -4.13, -4.13, -8.36, -8.36]

    # # Gazebo + walkway
    # x = [3.62, 9.19, 11.09, 12.88, 14.12, 14.12, 12.88, 11.12, 9.87, 9.87, 8.55, 3.63]
    # y = [-2.78, -2.93, -1.23, -1.22, -0.02, 1.78, 3.03, 3.03, 1.78, -0.02, -1.12, -1.12]

    # Tree
    x = [9.12, 14.23, 12.62]
    y = [-8.15, -8.86, -4.87]

    x0, y0, A = find_centroid(x, y)
    print(f'The centroid is x0: {round(x0,3)} , y0: {round(y0,3)} with area of {round(A,3)}.')

    translation = {'x':0, 'y':11.1} #applied translation for the optimal path planner that takes only positive values
    dpi = 1000
    x_dpi_translated, y_dpi_translated, x0_dpi_translated, y0_dpi_translated = transform_for_processing(x, y, x0, y0, dpi, translation)

    print('=================== AFTER NORMALIZED TO USE IN OPTIMAL PLANNER ===================')
    print(f'x0: {x0_dpi_translated} , y0: {y0_dpi_translated}')

    print(x0_dpi_translated, end =' ')
    print(y0_dpi_translated, end =' ')
    for i in range(len(x_dpi_translated)):
        print(x_dpi_translated[i], end =' ')
        if i != len(x_dpi_translated) - 1:
            print(y_dpi_translated[i], end =' ')
        else:
            print(y_dpi_translated[i])
