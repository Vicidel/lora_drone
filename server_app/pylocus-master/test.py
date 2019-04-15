
import numpy as np

from pylocus.point_set import PointSet
from pylocus.algorithms import reconstruct_srls
from pylocus.simulation import create_noisy_edm, create_mask, create_weights
from pylocus.basics import mse, rmse
import math


from datetime import datetime



class drone_datapoints:
	x = 0
	y = 0

set_of_data = set()
set_v2 = []



### this for testing Pylocus
points = PointSet(N=5, d=2)
points.set_points('random')
print("point to localize:", points.points[0, :])
print("anchors:", points.points[1:, :])

std = 0.1
edm_noisy = create_noisy_edm(points.edm, noise=std)

mask = create_mask(points.N , method='none')
weights = create_weights(points.N, method='one')
weights = np.multiply(mask, weights)

points_estimated  = reconstruct_srls(edm_noisy, points.points, W=weights)
error = mse(points_estimated[0, :], points.points[0, :])

print("estimated point: {}, original point: {}, mse: {:2.2e}".format(points_estimated[0, :],
                                                                points.points[0, :],
                                                                error))



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