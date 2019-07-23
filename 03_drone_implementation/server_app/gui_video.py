#
# File: app.py
# Author: Victor Delafontaine
# Date: May 2019
# 
# Swisscom server app to create the commands sent to the drone.
# Also relays information to the GMaps API.
#


# misc dependencies
import os
import json
import struct
import math
import random
import numpy as np
import datetime as dt
import time

# localization package: https://github.com/kamalshadi/Localization
import localization as lx

# GMaps API database
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db as firebase_db

# conversion meters distance and GPS coord
import geopy
from geopy.distance import VincentyDistance



#########################################################################################
##################################  FIREBASE STORAGE  ###################################
#########################################################################################

# add position
def add_drone_maps(lat, lng, rel_alt, state):

	# add data
	ref_drone = firebase_db.reference('droneR')
	ref_drone.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'gui_video.py: store_GPS',
	    'timestamp': time.time(),
	    'altitude': 0,
	    'rel_alt': rel_alt,
	    'drone_id': 1,
	    'state': state,
	    'fsm': 0,
	    'battery': -1
	})

	# return string
	return 'Data added to Firebase'


# add estimation
def add_estimation_maps(lat, lng, radius, est_type):

	# push on Firebase
	ref_est = firebase_db.reference('estimate')
	ref_est.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'gui_video.py: add_estimation',
	    'radius': radius,
	    'type': est_type
	})

	return 'Success'


# add waypoint
def add_waypoint_maps(lat, lng):

	# push on Firebase
	ref_wayp = firebase_db.reference('waypointR')
	ref_wayp.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'gui_video.py: add_waypoint',
	    'drone_id': 1,
	    'id': "test"
	})

	return 'Success'


#########################################################################################
####################################  MISC FUNCTIONS  ###################################
#########################################################################################

# conversion between latlng and xy
def conversion_latlng_xy(lat, lng, drone_id):

	# get drone_id home
	home = None
	if drone_id==1:
		home = homeR
	elif drone_id==2:
		home = homeG
	else:
		home = homeB
	
	# math stuff for bearing 
	delta_lon = home.longitude - lng
	bearing_y = math.sin(delta_lon) * math.cos(home.latitude)
	bearing_x = math.cos(lat) * math.sin(home.latitude) - math.sin(lat) * math.cos(home.latitude) * math.cos(delta_lon)
	bearing = math.atan2(bearing_y, bearing_x)

	# distance through geopy
	dist = geopy.distance.vincenty((lat, lng), (home.latitude, home.longitude)).m

	# debug print
	#print("*******************************************************************")
	#print("*******************************************************************")
	#print("Conversion latlng->xy: latlng {} {}".format(lat, lng))
	#print("  dist {} bearing {}".format(dist, bearing*180/(math.pi/2)))
	#print("  xy {} {}".format(math.sin(bearing)*dist, -math.cos(bearing)*dist))
	#print("*******************************************************************")
	#print("*******************************************************************")

	# return result
	return math.sin(bearing)*dist + home.delta_x, -math.cos(bearing)*dist + home.delta_y


# conversion between xy and latlng
def conversion_xy_latlng(x, y, drone_id):

	# get drone_id home
	home = None
	if drone_id==1:
		home = homeR
	elif drone_id==2:
		home = homeG
	else:
		home = homeB
	
	# home to zero
	home_latlng = geopy.Point(home.latitude, home.longitude)
	dist_to_zero, bearing_to_zero = get_dist_bearing(-float(home.delta_x), -float(home.delta_y))
	zero_latlng = VincentyDistance(meters=dist_to_zero).destination(home_latlng, bearing_to_zero)

	# zero to drone
	distance, bearing = get_dist_bearing(x, y)
	latlng = VincentyDistance(meters=distance).destination(zero_latlng, bearing)

	# debug print
	#print("*******************************************************************")
	#print("*******************************************************************")
	#print("Conversion xy-->latlng: xy {} {}".format(x, y))
	#print("	 latlng {} {}".format(latlng.latitude, latlng.longitude))
	#print("*******************************************************************")
	#print("*******************************************************************")

	# return result
	return latlng.latitude, latlng.longitude


# function to get distance and bearing from x and y distance
def get_dist_bearing(dist_x, dist_y):
	# compute distance
	distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)

	# compute bearing
	if dist_y==0:
		bearing = 180 if dist_x < 0 else 0
	elif dist_x==0:
		bearing = 90 if dist_y > 0 else 270
	else:
		bearing = math.degrees(math.atan(dist_y/dist_x))
	
	# negative x modifier
	if dist_x < 0:
		bearing = -bearing-90
	else:
		bearing = -bearing+90

	# return
	return distance, bearing


#########################################################################################
######################################  MAIN CODE  ######################################
#########################################################################################

# create the link with Firebase
cred = credentials.Certificate('lora-drone-v2-firebase-adminsdk-velks-2ef74f77f9.json')		# for lora-drone-v2 Firebase
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://lora-drone-v2.firebaseio.com/'		# for lora-drone-v2 Firebase
})

# delete database entries
ref_drone  = firebase_db.reference('drone')
ref_home   = firebase_db.reference('home')
ref_netw   = firebase_db.reference('network')
ref_est    = firebase_db.reference('estimate')
ref_wayp   = firebase_db.reference('waypoint')
ref_node   = firebase_db.reference('node')
ref_droneR = firebase_db.reference('droneR')
ref_homeR  = firebase_db.reference('homeR')
ref_waypR  = firebase_db.reference('waypointR')
ref_drone.delete()
ref_home.delete()
ref_netw.delete()
ref_est.delete()
ref_wayp.delete()
ref_node.delete()
ref_droneR.delete()
ref_homeR.delete()
ref_waypR.delete()

# lat lng
node_lat = 46.52136
node_lng = 6.53595
network_lat = 46.52135
network_lng = 6.53591
takeoff_lat = 46.52134
takeoff_lng = 6.53614
wp1_lat = 46.52130
wp1_lng = 6.53725
wp2_lat = 46.52209
wp2_lng = 6.53546
wp3_lat = 46.52064
wp3_lng = 6.53543
land_lat = 46.52122
land_lng = 6.53596

# add network on Firebase
ref_network = firebase_db.reference('network')
ref_network.push({
	'lat': network_lat,
	'lng': network_lng,
    'sender': 'gui_video.py: add_network',
})

# add ground truth on Firebase
ref_node = firebase_db.reference('node')
ref_node.push({
	'lat': node_lat,
	'lng': node_lng,
	'nb_sat': 10,
	'hdop': 0,
	'speed': 0,
	'course': 0,
    'sender': 'gui_video.py: add_ground_truth',
})

# add home 
ref_home = firebase_db.reference('homeR')
ref_home.push({
	'lat': takeoff_lat,
	'lng': takeoff_lng,
	'altitude': 0,
	'delta_x': 0,
	'delta_y': 0,
	'delta_z': 0,
    'sender': 'gui_video.py: store_home',
    'timestamp': time.time(),
    'drone_id': 1
})

# add drone
add_drone_maps(takeoff_lat, takeoff_lng, 0, 'MANUAL')
time.sleep(5)

# change drone state
add_drone_maps(takeoff_lat, takeoff_lng, 0, 'OFFBOARD')
time.sleep(1)

# takeoff
add_waypoint_maps(takeoff_lat, takeoff_lng)
altitude = 0
while 1:
	altitude = altitude + random.random()/2+1.75
	add_drone_maps(takeoff_lat+random.random()/100000, takeoff_lng+random.random()/100000, altitude, 'OFFBOARD')
	if altitude > 7:
		break
	time.sleep(0.5)

# set drone speed
drone_speed = 4

# time to clear wp
time.sleep(1)

# go to first waypoint
add_waypoint_maps(wp1_lat, wp1_lng)
drone_lat = takeoff_lat
drone_lng = takeoff_lng
delta_lat = wp1_lat - drone_lat
delta_lng = wp1_lng - drone_lng
increment_lat = delta_lat / (100/drone_speed)
increment_lng = delta_lng / (100/drone_speed)
i = 0
while i<(100/drone_speed):
	altitude = altitude + 1
	if altitude > 19:
		altitude = 18.5+random.random()*2
	i = i+1
	drone_lat = drone_lat + increment_lat
	drone_lng = drone_lng + increment_lng
	add_drone_maps(drone_lat+random.random()/100000, drone_lng+random.random()/100000, altitude, 'OFFBOARD')
	time.sleep(0.5)

# time to clear wp
time.sleep(0.2)

# go to second waypoint
add_waypoint_maps(wp2_lat, wp2_lng)
delta_lat = wp2_lat - drone_lat
delta_lng = wp2_lng - drone_lng
increment_lat = delta_lat / (160/drone_speed)
increment_lng = delta_lng / (160/drone_speed)
i = 0
while i<(160/drone_speed):
	i = i+1
	drone_lat = drone_lat + increment_lat
	drone_lng = drone_lng + increment_lng
	add_drone_maps(drone_lat+random.random()/100000, drone_lng+random.random()/100000, 18.5+random.random()*2, 'OFFBOARD')
	time.sleep(0.5)
	if i==20:
		add_estimation_maps(46.52190, 6.53688, 0, 'temp')
	if i==27:
		add_estimation_maps(46.52201, 6.53671, 0, 'temp')
	if i==33:
		add_estimation_maps(46.52151, 6.53600, 0, 'temp')
	if i==40:
		add_estimation_maps(46.52120, 6.53618, 0, 'temp')

# time to clear wp
time.sleep(0.2)

# go to third waypoint
add_waypoint_maps(wp3_lat, wp3_lng)
delta_lat = wp3_lat - drone_lat
delta_lng = wp3_lng - drone_lng
increment_lat = delta_lat / (160/drone_speed)
increment_lng = delta_lng / (160/drone_speed)
i = 0
while i<(160/drone_speed):
	i = i+1
	drone_lat = drone_lat + increment_lat
	drone_lng = drone_lng + increment_lng
	add_drone_maps(drone_lat+random.random()/100000, drone_lng+random.random()/100000, 18.5+random.random()*2, 'OFFBOARD')
	time.sleep(0.5)
	if i==5:
		add_estimation_maps(46.52119, 6.53597, 0, 'temp')
	if i==11:
		add_estimation_maps(46.52123, 6.53575, 0, 'temp')
	if i==15:
		add_estimation_maps(46.52127, 6.53579, 0, 'temp')
	if i==21:
		add_estimation_maps(46.52130, 6.53591, 0, 'temp')
	if i==27:
		add_estimation_maps(46.52122, 6.53582, 0, 'temp')
	if i==33:
		add_estimation_maps(46.52119, 6.53580, 0, 'temp')
	if i==40:
		add_estimation_maps(land_lat, land_lng, 0, 'temp')

# time to clear wp
time.sleep(0.2)

# go to landing waypoint
add_waypoint_maps(land_lat, land_lng)
add_estimation_maps(land_lat, land_lng, 30, 'est1')
delta_lat = land_lat - drone_lat
delta_lng = land_lng - drone_lng
increment_lat = delta_lat / (80/drone_speed)
increment_lng = delta_lng / (80/drone_speed)
i = 0
while i<(80/drone_speed):
	i = i+1
	drone_lat = drone_lat + increment_lat
	drone_lng = drone_lng + increment_lng
	add_drone_maps(drone_lat+random.random()/100000, drone_lng+random.random()/100000, 18.5+random.random()*2, 'OFFBOARD')
	time.sleep(0.5)

# landing
altitude = 20
while 1:
	altitude = altitude - random.random()/2-1.75
	add_drone_maps(land_lat+random.random()/100000, land_lng+random.random()/100000, altitude, 'OFFBOARD')
	if altitude < 1:
		break
	time.sleep(0.5)
