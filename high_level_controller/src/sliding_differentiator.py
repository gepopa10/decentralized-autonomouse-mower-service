#!/usr/bin/env python3
import numpy as np

def slid_diff(measured_inp,variable1_inp,variable2_inp,dt_inp):
	k1 = 10.0
	k2 = 1.0
	diff = variable1_inp - measured_inp
	out_main = ( -k1 * np.sqrt(np.abs(diff)) * np.sign(diff) ) + variable2_inp
	D_variable1 = out_main
	D_variable2 = -k2 * np.sign(diff)
	out_variable1 = variable1_inp + (D_variable1 * dt_inp)
	out_variable2 = variable2_inp + (D_variable2 * dt_inp)

	return out_main,out_variable1,out_variable2

