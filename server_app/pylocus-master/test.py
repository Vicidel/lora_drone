
import numpy as np

from pylocus.point_set import PointSet
from pylocus.algorithms import reconstruct_srls
from pylocus.simulation import create_noisy_edm, create_mask, create_weights
from pylocus.basics import mse, rmse, get_edm
import math


from datetime import datetime

from mlat import MLAT

import localization as lx


def test_pylocus():


	print("###############PYLOCUS EXAMPLE###################")
	points = PointSet(N=5, d=2)
	points.set_points('random')
	print(points.points)
	print("point to localize:", points.points[0, :])
	print("anchors:", points.points[1:, :])

	std = 0.1
	edm_noisy = create_noisy_edm(points.edm, noise=std)
	print(edm_noisy)

	mask = create_mask(points.N , method='none')
	weights = create_weights(points.N, method='one')
	weights = np.multiply(mask, weights)
	print(weights)

	points_estimated  = reconstruct_srls(edm_noisy, points.points, W=weights)
	error = mse(points_estimated[0, :], points.points[0, :])
	print(points_estimated)

	print("estimated point: {}, original point: {}, mse: {:2.2e}".format(points_estimated[0, :],
	                                                                points.points[0, :],
	                                                                error))


	print("###############PYLOCUS IMPLEMENTATION#########################")
	tri_dataset = []
	class tri_datapoint:
		pos_x 		= 666
		pos_y 		= 666
		pos_z 		= 666
		rssi 		= 666
		esp 		= 666
		distance 	= 666

	data1 = tri_datapoint()
	data2 = tri_datapoint()
	data3 = tri_datapoint()

	data1.pos_x = 100
	data1.pos_y = 0
	data1.pos_z = 10
	data1.distance = 103
	tri_dataset.append(data1)

	data2.pos_x = -100
	data2.pos_y = 0
	data2.pos_z = 10
	data2.distance = 100
	tri_dataset.append(data2)

	data3.pos_x = 0
	data3.pos_y = 100
	data3.pos_z = 10
	data3.distance = 102
	tri_dataset.append(data3)

	print(tri_dataset)

	class pylocus_datapoint:
		N = 0
		d = 0
		points = np.zeros((1,1))
		edm = np.zeros((1,1))

	pylocus = pylocus_datapoint()
	pylocus.N = len(tri_dataset) 	# number of points
	pylocus.d = 2					# number of dimensions
	pylocus.points = np.zeros((pylocus.N, pylocus.d))
	pylocus.edm = np.zeros((pylocus.N, pylocus.N))
	#pylocus.points[0,0] = 0
	#pylocus.points[0,1] = 0		# first point is the one to localizae
	for i in range(0,pylocus.N):
		pylocus.points[i,0] = tri_dataset[i].pos_x
		pylocus.points[i,1] = tri_dataset[i].pos_y
	print(pylocus.points)

	pylocus.edm = get_edm(pylocus.points)
	print(pylocus.edm)

	mask = create_mask(points.N , method='none')
	weights = create_weights(points.N, method='one')
	weights = np.multiply(mask, weights)
	print(weights)

	points_est = reconstruct_srls(pylocus.edm, pylocus.points, W=weights)
	print(points_est)

def old_tests():

	#this for testing set of class
	a = drone_datapoints()
	a.x = 2
	a.y = 5
	a1 = drone_datapoints()
	a1.x = 3
	a1.y = 7
	a2 = drone_datapoints()
	a2.x = 12
	a2.y = 532

	print("data is {}".format(a))
	print("data.x is {}".format(a.x))

	set_of_data.add(a)
	set_of_data.add(a1)
	set_of_data.add(a2)

	print("size is {}".format(len(set_of_data)))

	for i in set_of_data:
		print("i {}, x{}, y{}".format(i, i.x, i.y))

	print(set_v2)
	print(type(set_v2))

	set_v2.append(a)
	set_v2.append(a1)

	print(set_v2)

	for i in range(0,len(set_v2)):
		print(set_v2[i].x)




	test = 16
	if test not in {10, 12, 15}:
		print("yes")
	else:
		print("no")




	timestamp = 1545730073
	datetime_ts = datetime.fromtimestamp(timestamp)

	print(datetime_ts)
	print(type(datetime_ts))

	adding = 2.5
	timestamp_2 = (datetime_ts- datetime(1970,1,1)).total_seconds()
	print(timestamp_2)



	f_a = 0.2011
	f_b = -0.0556
	esp = -90
	dist = f_a * math.exp(f_b*esp)
	print("dist is", dist)

def simple_data():
	print("###################SIMPLE DATASET###################")
	tri_dataset = []
	class tri_datapoint:
		pos_x 		= 666
		pos_y 		= 666
		pos_z 		= 666
		distance 	= 666

	data1 = tri_datapoint()
	data2 = tri_datapoint()
	data3 = tri_datapoint()

	data1.pos_x = 100
	data1.pos_y = 0
	data1.pos_z = 10
	data1.distance = 103
	tri_dataset.append(data1)

	data2.pos_x = -100
	data2.pos_y = 0
	data2.pos_z = 10
	data2.distance = 100
	tri_dataset.append(data2)

	data3.pos_x = 0
	data3.pos_y = 100
	data3.pos_z = 10
	data3.distance = 102
	tri_dataset.append(data3)

	print(tri_dataset)

	test_pylocus()

def loc_package():
	print("###################LOCALIZATION PACKAGE TEST###################")
	tri_dataset = []
	class tri_datapoint:
		pos_x 		= 666
		pos_y 		= 666
		pos_z 		= 666
		distance 	= 666

	data1 = tri_datapoint()
	data2 = tri_datapoint()
	data3 = tri_datapoint()

	data1.pos_x = 100
	data1.pos_y = 0
	data1.pos_z = 0
	data1.distance = 95
	tri_dataset.append(data1)

	data2.pos_x = -100
	data2.pos_y = 0
	data2.pos_z = 0
	data2.distance = 90
	tri_dataset.append(data2)

	data3.pos_x = 0
	data3.pos_y = 100
	data3.pos_z = 0
	data3.distance = 99
	tri_dataset.append(data3)

	print(tri_dataset)

	P = lx.Project(mode='2D', solver='CCA')

	for i in range(0, len(tri_dataset)):
		P.add_anchor('anchor_n{}'.format(i), (tri_dataset[i].pos_x, tri_dataset[i].pos_x))

	t, label = P.add_target()

	for i in range(0, len(tri_dataset)):
		t.add_measure('anchor_n{}'.format(i), tri_dataset[i].distance)

	print(P)
	print(t)

	P.solve()

	# Then the target location is:

	print(t.loc)
		