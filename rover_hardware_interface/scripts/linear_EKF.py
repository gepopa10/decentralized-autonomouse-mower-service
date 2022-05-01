#!/usr/bin/env python

import numpy as np
import math
from numpy.linalg import multi_dot
from numpy import linalg

A = np.zeros([3,3])
A[0][1] = 1.0
A[1][2] = 1.0

B = np.zeros([3,1])
B[2][0] = 1.0

C = np.zeros([1,3])
C[0][0] = 1.0

q = 1.0
r = 0.1
eta = 0.8
Q = np.eye(3)*q
R = r
R_inv = 1.0/R

def EKF_DIST(dist_inp,x_hat_inp,P_inp,dt_inp):

	u = 0.0
	y = dist_inp

	x_hat = x_hat_inp
	P = P_inp
	dt = 0.1 #0.1*dt_inp

	P_dot_1 = np.dot(A,P)
	P_dot_2 = np.dot(P,np.transpose(A))
	P_dot_3 = np.dot(Q,np.transpose(Q))
	P_dot_4 = multi_dot([P,np.transpose(C),C,P]) * -1.0 * R_inv
	P_dot = np.add(P_dot_1,P_dot_2)
	P_dot = np.add(P_dot,P_dot_3)
	P_dot = np.add(P_dot,P_dot_4)

	x_hat_dot_1 = np.dot(A,x_hat)
	x_hat_dot_2 = np.dot(C,x_hat) * -1.0
	x_hat_dot_3 = np.add(y,x_hat_dot_2)
	x_hat_dot_4 = eta * multi_dot([P,np.transpose(C),x_hat_dot_3]) * R_inv
	x_hat_dot = np.add(x_hat_dot_1,x_hat_dot_4)

	P = P + (P_dot*dt)
	x_hat = x_hat + (x_hat_dot*dt)

	return x_hat,P
