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

# server database 
from flask import Flask, Response, request, redirect, url_for, escape, jsonify, make_response
from flask_mongoengine import MongoEngine
from flask_cors import CORS
from itertools import chain

# GMaps API database
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db as firebase_db

# conversion meters distance and GPS coord
import geopy
from geopy.distance import VincentyDistance




#########################################################################################
####################################  APP PARAMETERS  ###################################
#########################################################################################

# bootstrap the app
app = Flask(__name__)
CORS(app)		# enable CORS things: https://developer.mozilla.org/en-US/docs/Web/HTTP/CORS/Errors/CORSMissingAllowOrigin

# check if running in the cloud and set MongoDB settings accordingly
if 'VCAP_SERVICES' in os.environ:
	vcap_services = json.loads(os.environ['VCAP_SERVICES'])
	mongo_credentials = vcap_services['mongodb-2'][0]['credentials']
	mongo_uri = mongo_credentials['uri']
else:
	mongo_uri = 'mongodb://localhost/db'

app.config['MONGODB_SETTINGS'] = {'host': mongo_uri, 'db':'testing', 'alias':'default'}

# bootstrap our app
db = MongoEngine(app)

# set the port dynamically with a default of 3000 for local development
port = int(os.getenv('PORT', '3000'))

# create the link with Firebase
#cred = credentials.Certificate('drone-3bd2a-firebase-adminsdk-6ju7o-272f41c754.json')		# for lora-drone Firebase
cred = credentials.Certificate('lora-drone-v2-firebase-adminsdk-velks-2ef74f77f9.json')		# for lora-drone-v2 Firebase
firebase_admin.initialize_app(cred, {
    #'databaseURL': 'https://drone-3bd2a.firebaseio.com/'		# for lora-drone Firebase
    'databaseURL': 'https://lora-drone-v2.firebaseio.com/'		# for lora-drone-v2 Firebase
})


#########################################################################################
#################################  CLASSES AND DATASETS  ################################
#########################################################################################

# class for LoRa messages
class LoRa_datapoint(db.Document):
	# common to GPS and no-GPS
	devEUI       = db.StringField(required=True)
	deviceType   = db.StringField()
	timestamp    = db.DateTimeField()
	time         = db.StringField()
	sp_fact 	 = db.IntField()
	channel 	 = db.StringField()
	sub_band 	 = db.StringField()
	gateway_id   = db.ListField(db.StringField())
	gateway_rssi = db.ListField(db.FloatField())
	gateway_snr  = db.ListField(db.FloatField())
	gateway_esp  = db.ListField(db.FloatField())
	tx_power     = db.FloatField()

	# only for no-GPS method
	real_dist 	 = db.IntField()

	# only for GPS method
	gps_lat 	 = db.FloatField()
	gps_lon 	 = db.FloatField()
	gps_sat 	 = db.IntField()
	gps_hdop 	 = db.FloatField()
	gps_speed 	 = db.FloatField()
	gps_course 	 = db.IntField()

# class for drone messages 
class drone_datapoint:
	pos_x 		 = 666
	pos_y 		 = 666
	pos_z 		 = 666
	lat 		 = 666
	lng 		 = 666
	alt 		 = 666
	timestamp 	 = 666
	time 		 = ''
	payload 	 = ''
	state  		 = 666
	drone_id     = 1

# class for trilateration data	 
class tri_datapoint:
	pos_x 		= 666
	pos_y 		= 666
	pos_z 		= 666
	esp 		= 666
	rssi 		= 666
	distance 	= 666
	drone_id    = 666
	gw_id 		= 666

# class for the solution
class solution_datapoint:
	pos_x 		= 0
	pos_y 		= 0
	pos_z 		= 0

# class for the home
class home_datapoint:
	latitude    = 46.513381 	# center of football field
	longitude   = 6.563056 		# center of football field
	altitude    = 400 			# center of football field
	delta_x     = 0				# home is at zero
	delta_y     = 0				# home is at zero		
	delta_z     = 0				# home is at zero

# class for the estimation received from network
class network_datapoint:
	latitude    = 46.513381 	# center of football field
	longitude   = 6.563056 		# center of football field
	altitude    = 400 			# center of football field
	loc_radius  = 666
	alt_radius  = 666
	est_time    = 0
	est_ts		= 0

# datasets
drone_dataset 	   = []		# storage of the drone messages
tri_dataset        = []		# storage of the trilateration messages
tri_dataset_temp   = []		# storage of the temporary trilateration messages



#########################################################################################
###################################  LORA PARAMETERS  ###################################
#########################################################################################

# devices
gateway_printer   = '004A0DB4'
gateway_corner    = '004A1092'
gateway_logitech  = '004A1093'
victor_beacon     = '78AF580300000493'		# no gps
micha_beacon      = '78AF580300000485'		# gps

# time format
TIME_FORMAT       = "%Y-%m-%dT%H:%M:%S.%f+02:00"		# change that to +01:00 in winter time
TIME_FORMAT_QUERY = "%Y-%m-%dT%H:%M:%S"

# gateways on drones
gateway_id_RGB 	  = [gateway_corner, gateway_logitech, gateway_printer]



#########################################################################################
##################################  DRONE PARAMETERS  ###################################
#########################################################################################

# homes
homeR = home_datapoint()			# home of the R drone
homeG = home_datapoint()			# home of the G drone
homeB = home_datapoint()			# home of the B drone

# altitudes
flying_altitude   = 20
delta_fly_alt 	  = [-2, 0, 2]		# for obstacle avoidance, drones don't have same flying altitude
takeoff_altitude  = 5

# waypoint parameters
landing_radius    = 4    # circle around landing positions
hover_time 		  = 4
base_angle 		  = 0    # for one drone, to go to the 1st wp as fast as possible

# storage for state
current_state1 	  = 666
current_state2 	  = 666
current_state3 	  = 666

# number of estimations already made
nb_est_made       = 666

# for three drone: are the drones ready for next step
bool_drone1_ready = False
bool_drone2_ready = False
bool_drone3_ready = False

# for one or three drones: should the drone(s) start the algo (start from server)
bool_drone1_start = False
bool_drone2_start = False
bool_drone3_start = False

# for one or three drones: are the drone online and able to receive the offboard command
bool_drone1_online = False
bool_drone2_online = False
bool_drone3_online = False

# safety switch
gmaps_safety_switch  = 0		# 0 for none, 1 for kill, 2 for hover, 3 for RTL



#########################################################################################
###############################  LOCALIZATION PARAMETERS  ###############################
#########################################################################################

# solution storage
solution 		   = solution_datapoint()		# current solution
solution_temp 	   = solution_datapoint()		# temporary current solution

# localization parameters
loop_todo		  = 1

# network position
network = network_datapoint()
network_x 		  = 0 
network_y 		  = 0	 	# used parameters (will be changed)
network_z 		  = 0

# radius of waypoints around est
circle_radius     = 666     # used parameter, will be changed to the value below
circle_radius_v1  = 100	    # radius for first loop 
circle_radius_v2  = 50	    # radius for second loop
circle_radius_v3  = 20 		# radius for third and following loops

# uncertainties radius
est_uncertainty_net  = 150	# radius of uncertainty for the network
est_uncertainty1  = 50		# base uncertainty radius for the first estimation
est_uncertainty2  = 20		# base uncertainty radius for the second estimation
est_uncertainty3  = 10		# base uncertainty radius for the third estimation

# if we already made estimations before, compare it with new ones
bool_new_est_made = False



#########################################################################################
###################################  DEFAULT ROUTE  #####################################
#########################################################################################

# base route which just returns a string
@app.route('/')
def hello_world():
	return '<b>Welcome to the Drone application!</b>'



#########################################################################################
##################################  PRINTING ROUTE  #####################################
#########################################################################################

# print the drone dataset
@app.route('/print/drone_dataset', methods=['GET'])
def print_drone_dataset():

	# doing stuff to have a json...
	data_dict = {}
	for item in drone_dataset:
		data={"x": item.pos_x, "y": item.pos_y, "z": item.pos_z, "ts": (item.timestamp-dt.datetime(1970,1,1)).total_seconds(), "time": item.time, "state": item.state, "payload": item.payload, "id": item.drone_id}
		data_dict.update({"datapoint_"+str(drone_dataset.index(item)): data})

	# format as json and send
	return Response(json.dumps(data_dict), mimetype='application/json', headers={'Content-Disposition':'attachment;filename=query.json'})


# print the drone dataset temp
@app.route('/print/tri_dataset_temp', methods=['GET'])
def print_tri_dataset_temp():

	# doing stuff to have a json...
	data_dict = {}
	for item in tri_dataset_temp:
		data={"x": item.pos_x, "y": item.pos_y, "z": item.pos_z, "esp": item.esp, "rssi": item.rssi, "distance": item.distance, "drone": item.drone_id, "gateway": item.gw_id}
		data_dict.update({"datapoint_"+str(tri_dataset_temp.index(item)): data})

	# format as json and send
	return Response(json.dumps(data_dict), mimetype='application/json', headers={'Content-Disposition':'attachment;filename=query.json'})


# print the trilateration dataset
@app.route('/print/tri_dataset', methods=['GET'])
def print_tri_dataset():

	# doing stuff to have a json...
	data_dict = {}
	for item in tri_dataset:
		data={"x": item.pos_x, "y": item.pos_y, "z": item.pos_z, "esp": item.esp, "rssi": item.rssi, "distance": item.distance, "drone": item.drone_id, "gateway": item.gw_id}
		data_dict.update({"datapoint_"+str(tri_dataset.index(item)): data})

	# format as json and send
	return Response(json.dumps(data_dict), mimetype='application/json', headers={'Content-Disposition':'attachment;filename=query.json'})



#########################################################################################
##################################  FIREBASE STORAGE  ###################################
#########################################################################################

# store position in Firebase
@app.route('/firebase/store_GPS', methods=['POST'])
def store_current_coord():

	###################  DECODE PAYLOAD  ######################
	print("!!!!!!!!! GPS received from drone !!!!!!!!!")
	
	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the JSON received
	print("JSON received:")
	print(j)

	# parse received data
	lat         = float(j['lat'])
	lng         = float(j['lon'])
	alt         = float(j['alt'])
	rel_alt     = float(j['rel'])
	drone_id    = int(j['id'])
	state       = str(j['state'])
	fsm_state   = str(j['fsm'])
	r_timestamp = dt.datetime.utcfromtimestamp(float(j['ts']))

	# push on Firebase
	print("Pushing new drone position on Firebase {} {}".format(lat, lng))

	if drone_id==1:
		ref_drone = firebase_db.reference('droneR')
	if drone_id==2:
		ref_drone = firebase_db.reference('droneG')
	if drone_id==3:
		ref_drone = firebase_db.reference('droneB')

	ref_drone.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'app.py: store_GPS',
	    'timestamp': (r_timestamp - dt.datetime(1970,1,1)).total_seconds(),
	    'altitude': alt,
	    'rel_alt': rel_alt,
	    'drone_id': drone_id,
	    'state': state,
	    'fsm': fsm_state
	})

	# return string
	return 'Data added to Firebase'


# store home position in Firebase
@app.route('/firebase/store_home', methods=['POST'])
def store_current_home():

	###################  DECODE PAYLOAD  ######################
	print("!!!!!!!!! Home received from drone !!!!!!!!!")
	
	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the JSON received
	print("JSON received:")
	print(j)

	# parse received data
	r_lat       = float(j['lat'])
	r_lng       = float(j['lon'])
	r_alt       = float(j['alt'])
	r_dx        = float(j['dx'])
	r_dy        = float(j['dy'])
	r_dz        = float(j['dz'])
	r_drone_id  = int(j['id'])
	r_timestamp = dt.datetime.utcfromtimestamp(float(j['ts']))


	# add in server memory
	global homeR, homeG, homeB
	if r_drone_id==1:
		homeR.latitude  = r_lat
		homeR.longitude = r_lng
		homeR.altitude  = r_alt
		homeR.delta_x   = r_dx
		homeR.delta_y   = r_dy
		homeR.delta_z   = r_dz
		homeR.drone_id  = r_drone_id
	if r_drone_id==2:
		homeG.latitude  = r_lat
		homeG.longitude = r_lng
		homeG.altitude  = r_alt
		homeG.delta_x   = r_dx
		homeG.delta_y   = r_dy
		homeG.delta_z   = r_dz
		homeG.drone_id  = r_drone_id
	if r_drone_id==3:
		homeB.latitude  = r_lat
		homeB.longitude = r_lng
		homeB.altitude  = r_alt
		homeB.delta_x   = r_dx
		homeB.delta_y   = r_dy
		homeB.delta_z   = r_dz
		homeB.drone_id  = r_drone_id


	# push on Firebase
	print("Pushing home position on Firebase: lat={}, lng={}, alt={}".format(r_lat, r_lng, r_alt))
	ref_home = None
	if r_drone_id==1:
		ref_home = firebase_db.reference('homeR')
	if r_drone_id==2:
		ref_home = firebase_db.reference('homeG')
	if r_drone_id==3:
		ref_home = firebase_db.reference('homeB')
	ref_home.push({
		'lat': r_lat,
		'lng': r_lng,
		'altitude': r_alt,
		'delta_x': r_dx,
		'delta_y': r_dy,
		'delta_z': r_dz,
	    'sender': 'app.py: store_home',
	    'timestamp': (r_timestamp - dt.datetime(1970,1,1)).total_seconds(),
	    'drone_id': r_drone_id
	})

	# return string
	return 'Data added to Firebase and Swisscom server'


# empty Firebase when starting new run
@app.route('/firebase/empty', methods=['POST'])
def empty_firebase():

	print("!!!!!!!!! Deleting Firebase !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# drone id received
	r_drone_id  = int(j['id'])

	# delete database entries
	ref_drone  = firebase_db.reference('drone')
	ref_home   = firebase_db.reference('home')
	ref_netw   = firebase_db.reference('network')
	ref_est    = firebase_db.reference('estimate')
	ref_wayp   = firebase_db.reference('waypoint')
	ref_drone.delete()
	ref_home.delete()
	ref_netw.delete()
	ref_est.delete()
	ref_wayp.delete()

	# based on drone_id, do stuff
	if r_drone_id==1:
		ref_droneR = firebase_db.reference('droneR')
		ref_homeR  = firebase_db.reference('homeR')
		ref_waypR  = firebase_db.reference('waypointR')
		ref_droneR.delete()
		ref_homeR.delete()
		ref_waypR.delete()
	elif r_drone_id==2:
		ref_droneG = firebase_db.reference('droneG')
		ref_homeG  = firebase_db.reference('homeG')
		ref_waypG  = firebase_db.reference('waypointG')
		ref_droneG.delete()
		ref_homeG.delete()
		ref_waypG.delete()
	elif r_drone_id==3:
		ref_droneB = firebase_db.reference('droneB')
		ref_homeB  = firebase_db.reference('homeB')
		ref_waypB  = firebase_db.reference('waypointB')
		ref_droneB.delete()
		ref_homeB.delete()
		ref_waypB.delete()

	# return
	return 'Success'


# add estimation to maps
def add_estimation_maps(pos_x, pos_y, radius, est_type):

	# convert in latlng
	lat, lng = conversion_xy_latlng(pos_x, pos_y, 1)

	# push on Firebase
	ref_est = firebase_db.reference('estimate')
	ref_est.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'app.py: add_estimation',
	    'radius': radius,
	    'type': est_type
	})

	return 'Success'


# add waypoint on map
def add_waypoint_maps(pos_x, pos_y, drone_id, id_string):

	# convert in latlng
	lat, lng = conversion_xy_latlng(pos_x, pos_y, drone_id)

	# push on Firebase
	if drone_id==1:
		ref_wayp = firebase_db.reference('waypointR')
	if drone_id==2:
		ref_wayp = firebase_db.reference('waypointG')
	if drone_id==3:
		ref_wayp = firebase_db.reference('waypointB')

	ref_wayp.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'app.py: add_waypoint',
	    'drone_id': drone_id,
	    'id': id_string
	})

	return 'Success'


# change network estimation
def add_network_maps(pos_x, pos_y):

	# convert in latlng
	lat, lng = conversion_xy_latlng(pos_x, pos_y, 1)

	# push on Firebase
	ref_network = firebase_db.reference('network')
	ref_network.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'app.py: add_network',
	})

	return 'Success'


# add real node on maps
def add_ground_truth(lat, lng, nb_sat, hdop, speed, course):

	# push on Firebase
	ref_node = firebase_db.reference('node')
	ref_node.push({
		'lat': lat,
		'lng': lng,
		'nb_sat': nb_sat,
		'hdop': hdop,
		'speed': speed,
		'course': course,
	    'sender': 'app.py: add_ground_truth',
	})

	return 'Success'



#########################################################################################
###################################  TRILATERATION  #####################################
#########################################################################################

# converts the signal data to a distance
def function_signal_to_distance(esp, rssi):

	# exponential coefficents
	a_ESP  = 0.1973
	b_ESP  = -0.0902
	a_RSSI = 0.2189
	b_RSSI = -0.0894

	# compute based on ESP or RSSI
	distance = a_ESP * math.exp(b_ESP * esp)
	#distance = a_RSSI * math.exp(b_RSSI * rssi)

	# return the distance
	return distance


# uses the multilateration package on the input dataset
def trilateration(dataset):
	# this function uses the localization package by Kamal Shadi:
	# https://github.com/kamalshadi/Localization

	# print
	print("LOC: Starting multilateration with {} anchors".format(len(dataset)))

	# create new localization project using CCA method (centroid method) or LSE (least square error)
	P = lx.Project(mode='2D', solver='LSE')

	# create anchors and distances
	for i in range(0, len(dataset)):
		P.add_anchor('anchor_n{}'.format(i), (dataset[i].pos_x, dataset[i].pos_y))
	t, label = P.add_target()
	for i in range(0, len(dataset)):
		t.add_measure('anchor_n{}'.format(i), dataset[i].distance)

	# solve the problem
	P.solve()
	print("LOC: Found solution at x={}, y={}, z={}".format(t.loc.x, t.loc.y, t.loc.z))
	return t.loc.x, t.loc.y, t.loc.z


# find matching LoRa message from timestamp
def find_lora_match_ts(ts):

	# get timestamp interval 
	timestamp = (ts - dt.datetime(1970,1,1)).total_seconds() + 7200  	# +7200 for +2h in summer
	start = dt.datetime.fromtimestamp(timestamp-3)
	end = dt.datetime.fromtimestamp(timestamp+3)

	# find best message
	return LoRa_datapoint.objects(timestamp__lt=end,timestamp__gt=start).first()


# find matching LoRa message from gateway
def find_lora_match_gateway(lora_message, gateway_id):

	# loop all gateway IDs in list
	for index, gateway_id_looper in enumerate(lora_message.gateway_id):

		# checks if the gateway id is the one on the corresponding drone
		if str(gateway_id_looper) == gateway_id:
			return index
		else:
			continue

	# default return if function was not finished before
	return 666


# returns a tri_datapoint based on matching data
def match_tri_datapoint(timestamp, pos_x, pos_y, pos_z, drone_id, payload):

	# ignore unrelevant states
	if payload!='hover' and payload!='data':
		print("TRI: ignored because payload is not 'hover' or 'data' (payload: {})".format(payload))
		return None

	# find matching LoRa message from timestamp
	lora_message = find_lora_match_ts(timestamp)

	# check if we have a LoRa message with this timestamp
	if lora_message is None:
		# no datapoint exists with this timestamp
		print("TRI: no matching LoRa datapoint for timestamp {}".format((timestamp - dt.datetime(1970,1,1)).total_seconds() + 7200))
		return None
	
	# next: match from gateway
	gateway_index = find_lora_match_gateway(lora_message, gateway_id_RGB[drone_id-1])

	# check if the message contains the correct gateway
	if gateway_index==666:
		# no datapoint exists with this gateway id
		print("TRI: no matching LoRa datapoint for gateway {}".format(gateway_id_RGB[drone_id-1]))
		return None

	# check signal strength 
	if float(lora_message.gateway_rssi[gateway_index])<-100:
		# too small RSSI/ESP
		print("TRI: too low RSSI to use (<-100)")
		return None

	# create tri datapoint
	datapoint = tri_datapoint()

	# save drone data
	datapoint.pos_x 	= float(pos_x)
	datapoint.pos_y 	= float(pos_y)
	datapoint.pos_z 	= float(pos_z)

	# save LoRa data 
	datapoint.esp 	    = float(lora_message.gateway_esp[gateway_index])
	datapoint.rssi 	    = float(lora_message.gateway_rssi[gateway_index])
	datapoint.distance 	= float(function_signal_to_distance(datapoint.esp, datapoint.rssi))

	# add info
	datapoint.drone_id  = drone_id
	datapoint.gw_id     = gateway_id_RGB[drone_id-1]

	# return tri_datapoint
	print("TRI: datapoint x{0:.2f} y{0:.2f} d{0:.1f}".format(datapoint.pos_x, datapoint.pos_y, datapoint.distance))
	return datapoint


# returns the uncertainty of the trilateration
def tri_get_uncertainty(tri_dataset, x_est, y_est, z_est):

	# storage for max difference with true estimation
	max_distance = 0

	# do it five times
	for i in range(0,4):

		# modified dataset
		dataset = tri_dataset

		# modify tri_dataset
		for j in range(0, len(dataset)):

			# new distance with added +std in first half, -std in second half
			if j<len(dataset)/2:
				dataset[j].distance = float(function_signal_to_distance(dataset[j].esp+2.5, dataset[j].rssi+2.5))
			else:
				dataset[j].distance = float(function_signal_to_distance(dataset[j].esp-2.5, dataset[j].rssi-2.5))

		# do new tri
		x_est2, y_est2, z_est2 = trilateration(dataset)

		# compare with maximum distance
		delta_x = x_est - x_est2
		delta_y = y_est - y_est2
		max_distance = max(max_distance, math.sqrt(delta_x*delta_x + delta_y*delta_y))

	# return
	return max_distance*5 		# safety factor


# matches Lora and drone, then does trilateration on it
def trilateration_main():

	# global
	global tri_dataset, drone_dataset

	# clear tri_dataset
	tri_dataset = []
	
	# matches LoRa and drone datasets
	print("TRI: drone dataset is of length {}".format(len(drone_dataset)))
	for i in range(0, len(drone_dataset)):

		# match LoRa and drone (if possible)
		tri_datapoint = match_tri_datapoint(drone_dataset[i].timestamp, drone_dataset[i].pos_x, drone_dataset[i].pos_y, drone_dataset[i].pos_z, drone_dataset[i].drone_id, drone_dataset[i].payload)

		# add it if not None
		if tri_datapoint is not None:
			tri_dataset.append(tri_datapoint)

	# test if empty (forgot to start LoRa node?)
	if len(tri_dataset) == 0:
		print("ERROR: empty trilateration dataset (didn't start LoRa node?)")
		return 0, 0, 0, 666

	# test if enough points
	elif len(tri_dataset) < 3:
		print("ERROR: not enough datapoint for trilateration")
		return 0, 0, 0, 666

	# all good
	else:
		# call trilateration function
		pos_x_est, pos_y_est, pos_z_est = trilateration(tri_dataset)
		uncertainty = tri_get_uncertainty(tri_dataset, pos_x_est, pos_y_est, pos_z_est)

		# return result
		return pos_x_est, pos_y_est, pos_z_est, uncertainty


# does trilateration on tri_dataset_temp
def trilateration_main_temp():

	# global
	global tri_dataset_temp

	# test if empty (forgot to start LoRa node?)
	if len(tri_dataset_temp) == 0:
		print("ERROR: empty trilateration dataset (didn't start LoRa node?)")
		return 0, 0, 0

	# test if enough points
	elif len(tri_dataset_temp) < 3:
		print("ERROR: not enough datapoint for trilateration")
		return 0, 0, 0

	# all good
	else:
		# call trilateration function
		pos_x_est, pos_y_est, pos_z_est = trilateration(tri_dataset_temp)

		# return result
		return pos_x_est, pos_y_est, pos_z_est



#########################################################################################
#####################################  PARAMETERS  ######################################
#########################################################################################

# batch edit all parameters from POST
@app.route('/param/change', methods=['POST'])
def param_change():
	print("!!!!!!!!! New parameters estimate received from POST !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# set network estimate
	global network_x, network_y, network_z
	network_x = float(j['pos_x'])
	network_y = float(j['pos_y'])
	network_z = float(j['pos_z'])

	# add on map
	add_network_maps(network_x, network_y)

	# set network estimate
	global circle_radius_v1, circle_radius_v2
	circle_radius_v1 = float(j['radius_v1'])
	circle_radius_v2 = float(j['radius_v2'])

	# set hovering time
	global hover_time
	hover_time = float(j['hover_time'])

	# set altitudes
	global flying_altitude, takeoff_altitude
	flying_altitude = float(j['flight'])
	takeoff_altitude = float(j['takeoff'])

	# set number of loops
	global loop_todo
	loop_todo = float(j['loop_todo'])

	return "All parameters changed"


# batch edit all parameters from GMAPS
@app.route('/param/change_from_maps', methods=['POST'])
def param_change_gmaps():
	print("!!!!!!!!! New parameters estimate received from GMAPS !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# set network estimate
	global circle_radius_v1, circle_radius_v2, circle_radius_v3
	circle_radius_v1 = float(j['rad1'])
	circle_radius_v2 = float(j['rad2'])
	circle_radius_v3 = float(j['rad3'])

	# set hovering time
	global hover_time
	hover_time = float(j['hover'])

	# set number of loops
	global loop_todo
	loop_todo = float(j['loop_todo'])

	# set altitudes
	global flying_altitude, takeoff_altitude
	flying_altitude = float(j['flight'])
	takeoff_altitude = float(j['takeoff'])

	return "All parameters changed"


# batch print all parameters
@app.route('/param/print', methods=['GET'])
def param_print():

	# create dict
	data={'network':{
			'x': network_x,
			'y': network_y,
			'z': network_z,
		},
		'solution':{
			'x': solution.pos_x,
			'y': solution.pos_y,
			'z': solution.pos_z,
		},
		'radius':{
			'v1': circle_radius_v1,
			'v2': circle_radius_v2,
			'v3': circle_radius_v3,
		},
		'altitude':{
			'flying': flying_altitude,
			'takeoff': takeoff_altitude,
		},
		'hovering time': hover_time,
		'loops to do': loop_todo,
		'drone ready':{
			'droneR': bool_drone1_ready,
			'droneG': bool_drone2_ready,
			'droneB': bool_drone3_ready,
		},
		'drone online':{
			'droneR': bool_drone1_online,
			'droneG': bool_drone2_online,
			'droneB': bool_drone3_online,
		},
		'drone start':{
			'droneR': bool_drone1_start,
			'droneG': bool_drone2_start,
			'droneB': bool_drone3_start,
		}
	}

	# return	
	return Response(json.dumps(data), mimetype='application/json', headers={'Content-Disposition':'attachment;filename=query.json'})


# change the gateway ids
@app.route('/param/gateways', methods=['POST'])
def change_gateways():
	print("!!!!!!!!! New gateways ID received from GMAPS !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# set gateway IDs
	global gateway_id_RGB
	gateway_id_RGB[0] = str(j['gatewayR'])
	gateway_id_RGB[1] = str(j['gatewayG'])
	gateway_id_RGB[2] = str(j['gatewayB'])

	return "Gateway IDs changed"



#########################################################################################
################################  LORA NETWORK ESTIMATE  ################################
#########################################################################################


# to set the network estimation via Postman
@app.route('/lora/network_estimate', methods=['POST'])
def lora_network_est():
	print("!!!!!!!!! Network estimate received from POST (xy) !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the coordinates received
	print("Positions received: x={}, y={}, z={}".format(j['pos_x'], j['pos_y'], j['pos_z']))

	# set network estimate
	global network_x, network_y, network_z
	network_x = float(j['pos_x'])
	network_y = float(j['pos_y'])
	network_z = float(j['pos_z'])

	# add on map
	add_network_maps(network_x, network_y)

	# success
	return 'Network estimate positions set at {} {} {}'.format(j['pos_x'], j['pos_y'], j['pos_z'])


# to set the network estimation via server
@app.route('/lora/network_estimate_latlng', methods=['POST'])
def lora_network_est_latlng():
	print("!!!!!!!!! Network estimate received from POST  (latlng) !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the coordinates received
	print("Coordinates received: lat={}, lng={}".format(j['lat'], j['lng']))

	# convert in x, y
	x, y = conversion_latlng_xy(float(j['lat']), float(j['lng']), 1)
	print("Position computed: x={}, y={}".format(x, y))

	# add on map
	add_network_maps(x, y)

	# set network estimate
	global network_x, network_y, network_z
	network_x = x
	network_y = y
	network_z = 0

	# success
	return 'Network estimate positions set at {} {} {}'.format(x, y, 0)


# get network estimate
@app.route('/lora/compute_network_est', methods=['GET'])
def lora_net_est():

	# test if we received a location message at one point
	if network.loc_radius == 666:

		# nothing was ever set...
		print('No location data was received since the server started...')
		return 'No location received...'
	else:

		# display in log the coordinates received
		print("Last coordinates received from server: lat={}, lng={}".format(network.latitude, network.longitude))

		# convert in x, y
		x, y = conversion_latlng_xy(network.latitude, network.longitude, 1)

		# add on map
		add_network_maps(x, y)

		# set network estimate
		global network_x, network_y, network_z
		network_x = x
		network_y = y
		network_z = 0

		# return 
	 	return 'Received location: lat={}, lng={}'.format(network.latitude, network.longitude)



#########################################################################################
#####################################  DRONE STATUS  ####################################
#########################################################################################

# to set the drone ready manually for testing with three drones, but only one in Gazebo
@app.route('/param/drone_ready', methods=['POST'])
def param_drone_ready():
	print("!!!!!!!!! Drone status for next step received from POST !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the param received
	print("Drone status received: 1={}, 2={}, 3={}".format(j['bool_drone1_ready'], j['bool_drone2_ready'], j['bool_drone3_ready']))

	# set drone status
	global bool_drone1_ready, bool_drone2_ready, bool_drone3_ready
	bool_drone1_ready = int(j['bool_drone1_ready'])
	bool_drone2_ready = int(j['bool_drone2_ready'])
	bool_drone3_ready = int(j['bool_drone3_ready'])

	# success
	return 'Drones set at desired states'


# to start the drone from the server 
@app.route('/param/drone_start', methods=['POST'])
def param_drone_start():
	print("!!!!!!!!! Drone start received from POST !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the param received
	print("Drone start received: d1={}, d2={}, d3={}".format(j['bool_drone1_start'], j['bool_drone2_start'], j['bool_drone3_start']))

	# set drone status
	global bool_drone1_start, bool_drone2_start, bool_drone3_start
	bool_drone1_start = int(j['bool_drone1_start'])
	bool_drone2_start = int(j['bool_drone2_start'])
	bool_drone3_start = int(j['bool_drone3_start'])

	# return string
	return 'Drones starting modes set'


# to kill the drone from the server 
@app.route('/param/drone_kill', methods=['POST'])
def param_drone_kill():
	print("!!!!!!!!! Safety switch received from POST !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the param received
	print("Drone kill received: {}".format(j['kill']))

	# set drone status
	global gmaps_safety_switch
	gmaps_safety_switch  = int(j['kill'])

	# greys the buttons
	global bool_drone1_online, bool_drone2_online, bool_drone3_online
	if gmaps_safety_switch==True:
		bool_drone1_online = False
		bool_drone2_online = False
		bool_drone3_online = False

	# return string
	return 'Drones safety switch set'


# called from the drone to know if it should start
@app.route('/param/check_offboard', methods=['POST'])
def param_drone_for_takeoff():
	print("!!!!!!!!! Drone is checking takeoff !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# global variables to set drone as online (if asking for permission to takeoff, drone is online)
	global bool_drone1_online, bool_drone2_online, bool_drone3_online

	# display info 
	drone_id = int(j['id'])
	print("Asking for drone {}, params are d1={}, d2={}, d3={}".format(drone_id, bool_drone1_start, bool_drone2_start, bool_drone3_start))

	# for each drone, return Y(es) or (N)o
	if drone_id == 1:
		bool_drone1_online = True
		if bool_drone1_start:
			return 'Y: drone {} ready for takeoff'.format(drone_id)
		else:
			return 'N: drone {} not ready for takeoff'.format(drone_id)
	elif drone_id == 2:
		bool_drone2_online = True
		if bool_drone2_start:
			return 'Y: drone {} ready for takeoff'.format(drone_id)
		else:
			return 'N: drone {} not ready for takeoff'.format(drone_id)
	elif drone_id == 3:
		bool_drone3_online = True
		if bool_drone3_start:
			return 'Y: drone {} ready for takeoff'.format(drone_id)
		else:
			return 'N: drone {} not ready for takeoff'.format(drone_id)
	else:
		print('ERROR: drone number unknown ({})'.format(drone_id))
		return 'E: drone number unknown ({})'.format(drone_id)
	

# called from the drone for kill switch
@app.route('/param/check_kill', methods=['POST'])
def param_check_kill():
	print("!!!!!!!!! Drone is checking kill !!!!!!!!!")

	# return directly kill switch status
	return "{}".format(gmaps_safety_switch)


# called by gmaps to check if drones online to grey/ungrey buttons
@app.route('/param/check_online', methods=['POST'])
def param_check_online():

	# create dict
	data = {'droneR': bool_drone1_online, 'droneG': bool_drone2_online, 'droneB': bool_drone3_online}

	# response json
	return Response(json.dumps(data), mimetype='application/json', headers={'Content-Disposition':'attachment;filename=query.json'})



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


# function for decoding payload
def bitshift(payload, lastbyte):
	# just return value
	return 8*(payload-lastbyte-1)


# function to compute latlng based on latlng and distance
def latlng_shift(origin_lat, origin_lng, dist_x, dist_y):

	# create point with latlng
	origin = geopy.Point(origin_lat, origin_lng)

	# compute distance and bearing of distance
	distance, bearing = get_dist_bearing(dist_x, dist_y)
	destination = VincentyDistance(meters=distance).destination(origin, bearing)

	# return
	return destination.lat, destination.lng



#########################################################################################
#####################################  LORA ROUTE  ######################################
#########################################################################################


# Swisscom LPN listener to POST from actility
@app.route('/lora/receive_message', methods=['POST'])
def lora_receive():
	print("!!!!!!!!! Data received from ThingPark !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the JSON received
	print("JSON received:")
	print(j)

	# message is data
	if 'DevEUI_uplink' in j:

		# parse communication parameters
		r_deveui    = j['DevEUI_uplink']['DevEUI']
		r_time      = j['DevEUI_uplink']['Time']
		r_timestamp = dt.datetime.strptime(r_time, TIME_FORMAT)
		r_sp_fact   = j['DevEUI_uplink']['SpFact']
		r_channel   = j['DevEUI_uplink']['Channel']
		r_band      = j['DevEUI_uplink']['SubBand']
		r_devtype   = "tuino-v3"
		r_tx_power  = j['DevEUI_uplink']['TxPower']

		# decode payload based on which device is sending
		payload_int = int(j['DevEUI_uplink']['payload_hex'], 16)	# 16 as payload info in hexadecimal
		size_payload = 20
		if r_deveui==victor_beacon:
			# real distance (for calibration measures)
			r_real_dist   = payload_int

			# empty GPS
			r_lat    = 666
			r_lon    = 666
			r_sat    = 666
			r_hdop   = 666
			r_speed  = 666
			r_course = 666

		elif r_deveui==micha_beacon:
			# empty real distance
			r_real_dist = 666

			# GPS stuff
			r_lat    = ((payload_int & 0x0000000000ffffffff0000000000000000000000) >> bitshift(size_payload,8))/10000000.0
			r_lon    = ((payload_int & 0x000000000000000000ffffffff00000000000000) >> bitshift(size_payload,12))/10000000.0
			r_sat    = ((payload_int & 0x00000000000000000000000000ff000000000000) >> bitshift(size_payload,13))
			r_hdop   = ((payload_int & 0x0000000000000000000000000000ffff00000000) >> bitshift(size_payload,15))
			r_speed  = ((payload_int & 0x00000000000000000000000000000000ff000000) >> bitshift(size_payload,16)) / 2
			r_course = ((payload_int & 0x0000000000000000000000000000000000ff0000) >> bitshift(size_payload,17)) * 2

			# add to firebase
			if r_lon != 0:
				print("Adding to Firebase as ground truth")
				add_ground_truth(r_lat, r_lon, r_sat, r_hdop, r_speed, r_course)
			else: 
				print("No GPS data, not adding to Firebase")

		# store only one gateway information or all gateways
		store_only_one = False
		unique_gateway = gateway_corner
		if store_only_one:
			# set gateways parameters for transmission arriving on multiple gateways
			g_id   = []
			g_rssi = []
			g_snr  = []
			g_esp  = []

			# store only metadata of gateway
			for item in j['DevEUI_uplink']['Lrrs']['Lrr']:
				if unique_gateway == item['Lrrid']:
					g_id.append(item['Lrrid'])
					g_rssi.append(item['LrrRSSI'])
					g_snr.append(item['LrrSNR'])
					g_esp.append(item['LrrESP'])
			
			# if drone gateway not detected
			if not g_id:
				print('information not received by set gateway')
				return 'ERROR: Datapoint not saved (not received by set gateway)'
		else:
			# set gateways parameters for transmission arriving on multiple gateways
			g_id   = []
			g_rssi = []
			g_snr  = []
			g_esp  = []

			#parse array of multiple gateways
			for item in j['DevEUI_uplink']['Lrrs']['Lrr']:
				g_id.append(item['Lrrid'])
				g_rssi.append(item['LrrRSSI'])
				g_snr.append(item['LrrSNR'])
				g_esp.append(item['LrrESP'])

		# create new datapoint with parsed data
		datapoint = LoRa_datapoint(devEUI=r_deveui, time=r_time, timestamp=r_timestamp, deviceType=r_devtype, 
			sp_fact=r_sp_fact, channel=r_channel, sub_band=r_band, gateway_id=g_id, gateway_rssi=g_rssi, 
			gateway_snr=g_snr, gateway_esp=g_esp, real_dist=r_real_dist, tx_power=r_tx_power, 
			gps_lat=r_lat, gps_lon=r_lon, gps_sat=r_sat, gps_hdop=r_hdop, gps_speed=r_speed, gps_course=r_course)

		# save it to database
		datapoint.save()
		print('SUCESS: new LoRa datapoint saved to database')

		# success
		return 'Datapoint DevEUI %s saved' %(r_deveui)

	# message is location
	else:

		# parse values received (see PDF for all info received)
		r_deveui    = j['DevEUI_location']['DevEUI']
		r_time      = j['DevEUI_location']['Time']
		r_timestamp = dt.datetime.strptime(r_time, TIME_FORMAT)
		r_lat 		= j['DevEUI_location']['DevLAT']
		r_lng 		= j['DevEUI_location']['DevLON']
		r_alt 		= j['DevEUI_location']['DevAlt']
		r_loc_rad   = j['DevEUI_location']['DevLocRadius']
		r_alt_rad   = j['DevEUI_location']['DevAltRadius']

		# set values 
		network.latitude   = r_lat
		network.longitude  = r_lng
		network.altitude   = r_alt
		network.loc_radius = r_loc_rad
		network.alt_radius = r_alt_rad
		network.est_time   = r_time
		network.est_ts     = r_timestamp

		# success
		return 'Localization at lat={} and lng={} saved'.format(r_lat, r_lng)


# querying the database and giving back a JSON file
@app.route('/lora/query', methods=['GET'])
def lora_query():
	query = request.args

	# to delete datapoint based on time
	if 'delete_time_point' in query and 'time' in query:
		deltime_start = dt.datetime.strptime(query['time'], TIME_FORMAT_QUERY) - dt.timedelta(seconds=2)
		deltime_end = dt.datetime.strptime(query['time'], TIME_FORMAT_QUERY) + dt.timedelta(seconds=2)
		LoRa_datapoint.objects(timestamp__lt=deltime_end, timestamp__gt=deltime_start).delete()
		return 'point deleted'

	# to delete datapoints based on time
	if 'delete_time_interval' in query and 'start' in query and 'end' in query:
		end = dt.datetime.strptime(query['end'], TIME_FORMAT_QUERY)
		start = dt.datetime.strptime(query['start'], TIME_FORMAT_QUERY)
		LoRa_datapoint.objects(timestamp__lt=end,timestamp__gt=start).delete()
		return 'time interval deleted'

	# defaults: only sf of 7 and in the previous year
	sf = 7
	start = dt.datetime.now() - dt.timedelta(days=365)  # begins selection one year ago
	end = dt.datetime.now() + dt.timedelta(hours=2)		# just in case we have timestamps in the future?

	# change start and end time
	if 'start' in query:
		start = dt.datetime.strptime(query['start'], TIME_FORMAT_QUERY)
	if 'end' in query:
		end = dt.datetime.strptime(query['end'], TIME_FORMAT_QUERY)

	# change SF to select
	if 'sf' in query:
		sf = int(query['sf'])

	datapoints = LoRa_datapoint.objects(timestamp__lt=end,timestamp__gt=start,sp_fact=sf).to_json()

	#return datapoints 		# for directly in browser
	return Response(datapoints,mimetype='application/json', 	# for automatic file download
		headers={'Content-Disposition':'attachment;filename=query.json'})


# output JSON as downloaded file, LoRa database
@app.route('/lora/json', methods=['GET'])
def lora_export_json():
	print('Exporting LoRa database as JSON')
	return Response(LoRa_datapoint.objects.to_json(),mimetype='application/json',
		headers={'Content-Disposition':'attachment;filename=database.json'})


# print JSON directly in browser, LoRa database
@app.route('/lora/json_print', methods=['GET'])
def lora_print_json():
	print('Printing LoRa database as JSON')
	return LoRa_datapoint.objects.to_json()



#########################################################################################
####################################  DRONE ROUTE  ######################################
#########################################################################################


# returns the angle in radian at which the first waypoint is located for flight optimization
def get_base_angle(pos_x, pos_y, network_x, network_y):

	# could have done that in code, but cleaner that way
	return math.atan2(network_y-pos_y, network_x-pos_x)+math.pi


# returns the waypoint based on drone id, total number of drone and state for classic method
def get_waypoint(drone_id, nb_drone, pos_x, pos_y, bool_continuous):

	# global variables
	global solution, solution_temp, circle_radius, nb_est_made, bool_new_est_made, base_angle

	# uncertainty computed
	est_uncertainty = 666

	# default is not landing waypoint
	bool_landing_waypoint = False

	# get state from global variable
	if drone_id==1:
		state = current_state1
	if drone_id==2:
		state = current_state2
	if drone_id==3:
		state = current_state3

	# one drone
	if nb_drone==1:

		# no need for multilateration
		if state==0:
			# get base angle
			base_angle = get_base_angle(pos_x, pos_y, solution.pos_x, solution.pos_y)

			# first
			wp_x = solution.pos_x + circle_radius*math.cos(base_angle)
			wp_y = solution.pos_y + circle_radius*math.sin(base_angle)
			wp_z = flying_altitude
		elif state%3==1:	# state=1/4/7/10...
			# second
			wp_x = solution.pos_x + circle_radius*math.cos(base_angle+2*math.pi/3)
			wp_y = solution.pos_y + circle_radius*math.sin(base_angle+2*math.pi/3)
			wp_z = flying_altitude
		elif state%3==2:	# state=2/5/8/11...
			# third
			wp_x = solution.pos_x + circle_radius*math.cos(base_angle+4*math.pi/3)
			wp_y = solution.pos_y + circle_radius*math.sin(base_angle+4*math.pi/3)
			wp_z = flying_altitude

		# need multilateration: state=3/6/9/12...
		elif state%3==0:
			# bool_continuous is False: classic mode, do multilateration
			if bool_continuous==False:

				# do multilateration and store position
				pos_x_est, pos_y_est, pos_z_est, uncertainty = trilateration_main()
				if pos_x_est == 0 and pos_y_est == 0 and pos_z_est == 0:
					# base uncertainty
					if state==3:
						est_uncertainty = est_uncertainty1		# state=3
					elif state==6:
						est_uncertainty = est_uncertainty2		# state=6
					else:
						est_uncertainty = est_uncertainty3		# state=9/12/15...

					# old position
					print("LOC: Reusing previous estimate")
				else:
					# computed uncertainty
					est_uncertainty = uncertainty

					# new position
					solution.pos_x = pos_x_est
					solution.pos_y = pos_y_est
					solution.pos_z = pos_z_est

			# bool_continuous is True: continuous mode, use solution_temp as solution
			else:

				# get solution as solution_temp or old one if nothing found
				if solution_temp.pos_x == 0 and solution_temp.pos_y == 0 and solution_temp.pos_z == 0:
					# base uncertainty
					if state==3:
						est_uncertainty = est_uncertainty1		# state=3
					elif state==6:
						est_uncertainty = est_uncertainty2		# state=6
					else:
						est_uncertainty = est_uncertainty3		# state=9/12/15...

					# old position
					print("LOC: Reusing previous estimate")
				else:
					# computed uncertainty
					est_uncertainty = tri_get_uncertainty(tri_dataset_temp, solution_temp.pos_x, solution_temp.pos_y, solution_temp.pos_z)

					# new position
					solution.pos_x = solution_temp.pos_x
					solution.pos_y = solution_temp.pos_y
					solution.pos_z = solution_temp.pos_z

			# increment estimations made
			nb_est_made = state/3
			
			# add estimate to map
			add_estimation_maps(solution.pos_x, solution.pos_y, est_uncertainty, 'est'+str(nb_est_made))
			
			# check if end is now
			if nb_est_made==loop_todo:
				# finish program, return landing waypoint above estimate
				wp_x = solution.pos_x
				wp_y = solution.pos_y
				wp_z = flying_altitude
				bool_landing_waypoint = True
			else:
				# program still running, new parameters
				if state==3:
					circle_radius = circle_radius_v2
					base_angle = get_base_angle(pos_x, pos_y, solution.pos_x, solution.pos_y)
				else:
					circle_radius = circle_radius_v3
					base_angle = get_base_angle(pos_x, pos_y, solution.pos_x, solution.pos_y)

				# first
				wp_x = solution.pos_x + circle_radius*math.cos(base_angle)
				wp_y = solution.pos_y + circle_radius*math.sin(base_angle)
				wp_z = flying_altitude

	# three drones
	if nb_drone==3:

		# if unknown drone number
		if drone_id>3 or drone_id<1:
			print("ERROR: unknown drone number (not 1/2/3)")
			wp_x = pos_x
			wp_y = pos_y
			wp_z = flying_altitude

		# first waypoints
		if state==0:
			if drone_id==1:
				# east of estimate
				wp_x = solution.pos_x + circle_radius*math.cos(0)
				wp_y = solution.pos_y + circle_radius*math.sin(0)
				wp_z = flying_altitude + delta_fly_alt[drone_id-1]
			elif drone_id==2:
				# north-west of estimate
				wp_x = solution.pos_x + circle_radius*math.cos(2*math.pi/3)
				wp_y = solution.pos_y + circle_radius*math.sin(2*math.pi/3)
				wp_z = flying_altitude + delta_fly_alt[drone_id-1]
			elif drone_id==3:
				# south-west of estimate
				wp_x = solution.pos_x + circle_radius*math.cos(4*math.pi/3)
				wp_y = solution.pos_y + circle_radius*math.sin(4*math.pi/3)
				wp_z = flying_altitude + delta_fly_alt[drone_id-1]

		# estimations
		if state>0:
			# bool_continuous is False: classic mode, do multilateration
			if bool_continuous==False:

				# do multilateration and store position
				pos_x_est, pos_y_est, pos_z_est, uncertainty = trilateration_main()
				if pos_x_est == 0 and pos_y_est == 0 and pos_z_est == 0:
					# base uncertainty
					if state==1:
						est_uncertainty = est_uncertainty1		# state=1
					elif state==2:
						est_uncertainty = est_uncertainty2		# state=2
					else:
						est_uncertainty = est_uncertainty3		# state=3/4/5...

					# old estimation
					print("LOC: Reusing previous estimate")
				else:
					# computed uncertainty
					est_uncertainty = uncertainty

					# new position
					solution.pos_x = pos_x_est
					solution.pos_y = pos_y_est
					solution.pos_z = pos_z_est

			# bool_continuous is True: continuous mode, use solution_temp as solution
			else:

				# get solution as solution_temp or old one if nothing found
				if solution_temp.pos_x == 0 and solution_temp.pos_y == 0 and solution_temp.pos_z == 0:
					# base uncertainty
					if state==1:
						est_uncertainty = est_uncertainty1		# state=1
					elif state==2:
						est_uncertainty = est_uncertainty2		# state=2
					else:
						est_uncertainty = est_uncertainty3		# state=3/4/5...

					# old position
					print("LOC: Reusing previous estimate")
				else:
					# computed uncertainty
					est_uncertainty = tri_get_uncertainty(tri_dataset_temp, solution_temp.pos_x, solution_temp.pos_y, solution_temp.pos_z)

					# new position
					solution.pos_x = solution_temp.pos_x
					solution.pos_y = solution_temp.pos_y
					solution.pos_z = solution_temp.pos_z

			# increment estimations made
			nb_est_made = state

			# add estimate to map
			add_estimation_maps(solution.pos_x, solution.pos_y, est_uncertainty, 'est'+str(nb_est_made))

			# check if end is now
			if nb_est_made==loop_todo:
				# finish program, return landing waypoint above estimate
				circle_radius = landing_radius
				bool_landing_waypoint = True
			else:
				# program still running, new parameters
				if state==1:
					circle_radius = circle_radius_v2
				else:
					circle_radius = circle_radius_v3

			# new waypoint (can be landing or next)
			if drone_id==1:
				wp_x = solution.pos_x + circle_radius*math.cos(0)
				wp_y = solution.pos_y + circle_radius*math.sin(0)
				wp_z = flying_altitude + delta_fly_alt[drone_id-1]
				bool_drone1_ready = False
			elif drone_id==2:
				wp_x = solution.pos_x + circle_radius*math.cos(2*math.pi/3)
				wp_y = solution.pos_y + circle_radius*math.sin(2*math.pi/3)
				wp_z = flying_altitude + delta_fly_alt[drone_id-1]
				bool_drone2_ready = False
			elif drone_id==3:
				wp_x = solution.pos_x + circle_radius*math.cos(4*math.pi/3)
				wp_y = solution.pos_y + circle_radius*math.sin(4*math.pi/3)
				wp_z = flying_altitude + delta_fly_alt[drone_id-1]
				bool_drone3_ready = False

	return wp_x, wp_y, wp_z, bool_landing_waypoint


# returns the return string
def get_return_string(payload, drone_id, nb_drone, pos_x, pos_y):

	# global
	global drone_dataset, drone_dataset_temp, tri_dataset, tri_dataset_temp
	global current_state1, current_state2, current_state3
	global solution, solution_temp, circle_radius, nb_est_made, bool_new_est_made
	global bool_drone1_ready, bool_drone2_ready, bool_drone3_ready


	# if it is not set during the next if's
	return_string = 'ERROR, return string not set'


	###################  COMMON PAYLOADS  ######################

	# switch to OFFBOARD mode done
	if payload=='offb':
		return_string = "Congrats on offboard mode!"

	# drone was just armed
	if payload=='arm':

		# add waypoint for takeoff to Firebase
		add_waypoint_maps(pos_x, pos_y, drone_id, 'takeoff')

		# return string with takeoff coordinates
		return_string = "Takeoff at current position: h{}".format(takeoff_altitude)

	# drone took off and is in the air
	if payload=='takeoff':

		# empty drone dataset
		print("Emptying drone and tri dataset temp for this mission")
		drone_dataset = []
		tri_dataset_temp = []

		# check if network est was set 
		if network_x==0 and network_y==0 and network_z==0:
			# no: solution is current position
			solution.pos_x = pos_x
			solution.pos_y = pos_y
			solution.pos_z = 0
			solution_temp.pos_x = pos_x
			solution_temp.pos_y = pos_y
			solution_temp.pos_z = 0
		else:
			# yes: solution is network estimate set
			solution.pos_x = network_x
			solution.pos_y = network_y
			solution.pos_z = 0
			solution_temp.pos_x = network_x
			solution_temp.pos_y = network_y
			solution_temp.pos_z = 0

		# set base parameters
		circle_radius = circle_radius_v1

		# set state at 0
		if drone_id==1:
			current_state1 = 0
		if drone_id==2:
			current_state2 = 0
		if drone_id==3:
			current_state3 = 0

		## all drones needs to be ready again
		bool_drone1_ready = False
		bool_drone2_ready = False
		bool_drone3_ready = False

		# set estimation made at 0
		nb_est_made = 0

		# get waypoint
		wp_x, wp_y, wp_z, bool_landing_waypoint = get_waypoint(drone_id, nb_drone, pos_x, pos_y, False)

		# save on map
		add_waypoint_maps(wp_x, wp_y, drone_id, 'WP')
		add_network_maps(solution.pos_x, solution.pos_y)		# point
		add_estimation_maps(solution.pos_x, solution.pos_y, est_uncertainty_net, 'network') 	# circle

		# return string with new coordinates
		return_string = "New waiipoint: x{} y{} z{}".format(wp_x, wp_y, wp_z)

	# drone is landing
	if payload=='land':
		return_string = "Congrats on mission!"


	###################  PAYLOADS FOR CLASSIC ONLY  ######################

	# (classic only) drone reached its previous (unknown) waypoint
	if payload=='wp_ok':

		# read old state to increment it
		if drone_id==1:
			current_state1 = current_state1 + 1
		if drone_id==2:
			current_state2 = current_state2 + 1
		if drone_id==3:
			current_state3 = current_state3 + 1

		# reset bool
		bool_new_est_made = False

		## all drones needs to be ready again
		bool_drone1_ready = False
		bool_drone2_ready = False
		bool_drone3_ready = False

		# send hover time
		return_string = "Wait at this position"

	# (classic only) drone is waiting for all drones ready
	if payload=='wait':

		# bool if ready for next waypoint
		bool_drone_can_start_collection = False

		# fill bool_ready
		if drone_id==1:
			bool_drone1_ready = True
		if drone_id==2:
			bool_drone2_ready = True
		if drone_id==3:
			bool_drone3_ready = True

		# for one drone, true
		if nb_drone==1:
			bool_drone_can_start_collection = True

		# for three drones: check if all drones have finished their hovering
		if nb_drone==3:
			if bool_drone1_ready == True and bool_drone2_ready == True and bool_drone3_ready == True:
				bool_drone_can_start_collection = True

		if bool_drone_can_start_collection:
			# can start hovering for data collection
			return_string = "Hover time set: h{}".format(hover_time)

			# for three drones, empty drone dataset to avoid unbalanced localization
			if nb_drone==3:
				print("Emptying drone dataset for current localization")
				drone_dataset = []
		else:
			# still waiting
			return_string = "Wait until other drones ready (d1:{}, d2:{} d3:{})".format(bool_drone1_ready, bool_drone2_ready, bool_drone3_ready)

	# (classic only) drone is hovering in position
	if payload=='hover':

		# get new waypoint
		wp_x, wp_y, wp_z, bool_landing_waypoint = get_waypoint(drone_id, nb_drone, pos_x, pos_y, False)

		# return string
		return_string = "Wait until finished hovering (h:{})".format(hover_time)

	# (classic only) getting the next waypoint
	if payload=='wait_next':

		# get new waypoint
		wp_x, wp_y, wp_z, bool_landing_waypoint = get_waypoint(drone_id, nb_drone, pos_x, pos_y, False)

		# return string
		if bool_landing_waypoint:
			return_string = "Land at position: x{} y{} z{}".format(wp_x, wp_y, wp_z)
			add_waypoint_maps(wp_x, wp_y, drone_id, 'landing')
		else:
			return_string = "New waiipoint: x{} y{} z{}".format(wp_x, wp_y, wp_z)
			add_waypoint_maps(wp_x, wp_y, drone_id, 'WP')


	###################  PAYLOADS FOR CONTINUOUS ONLY  ######################

	# (continuous only) drone reached its previous (unknown) waypoint
	if payload=='wp_ok_cont':

		# read old state to increment it
		if drone_id==1:
			current_state1 = current_state1 + 1
		if drone_id==2:
			current_state2 = current_state2 + 1
		if drone_id==3:
			current_state3 = current_state3 + 1

		# reset bool
		bool_new_est_made = False

		# get waypoint 
		wp_x, wp_y, wp_z, bool_landing_waypoint = get_waypoint(drone_id, nb_drone, pos_x, pos_y, True)

		# return string
		if bool_landing_waypoint:
			return_string = "Land at position: x{} y{} z{}".format(wp_x, wp_y, wp_z)
			add_waypoint_maps(wp_x, wp_y, drone_id, 'landing')
		else:
			return_string = "New waiipoint: x{} y{} z{}".format(wp_x, wp_y, wp_z)
			add_waypoint_maps(wp_x, wp_y, drone_id, 'WP')

	# (continuous only) drone received data, make a new estimation
	if payload=='data':

		# do multilateration and store position
		pos_x_est, pos_y_est, pos_z_est = trilateration_main_temp()
		if pos_x_est == 0 and pos_y_est == 0 and pos_z_est == 0:
			# old position
			print("LOC: Reusing previous estimate")
		else:
			# new position
			solution_temp.pos_x = pos_x_est
			solution_temp.pos_y = pos_y_est
			solution_temp.pos_z = pos_z_est

			# computed uncertainty
			uncertainty = tri_get_uncertainty(tri_dataset_temp, solution_temp.pos_x, solution_temp.pos_y, solution_temp.pos_z)

			# add estimate to map
			add_estimation_maps(solution_temp.pos_x, solution_temp.pos_y, uncertainty, 'temp')

	# (continuous and 3drones only) drone received data, needs a new waypoint based on estimation
	if payload=='data2':
		
		# do multilateration and store position
		pos_x_est, pos_y_est, pos_z_est = trilateration_main_temp()
		if pos_x_est == 0 and pos_y_est == 0 and pos_z_est == 0:
			# old position
			print("LOC: Reusing previous estimate")
		else:
			# new position
			solution_temp.pos_x = pos_x_est
			solution_temp.pos_y = pos_y_est
			solution_temp.pos_z = pos_z_est

			# computed uncertainty
			uncertainty = tri_get_uncertainty(tri_dataset_temp, solution_temp.pos_x, solution_temp.pos_y, solution_temp.pos_z)

			# add estimate to map
			add_estimation_maps(solution_temp.pos_x, solution_temp.pos_y, uncertainty, 'temp')

		# get waypoint 
		wp_x, wp_y, wp_z, bool_landing_waypoint = get_waypoint(drone_id, nb_drone, pos_x, pos_y, True)

		# return string
		if bool_landing_waypoint:
			return_string = "Land at position: x{} y{} z{}".format(wp_x, wp_y, wp_z)
			add_waypoint_maps(wp_x, wp_y, drone_id, 'landing')
		else:
			return_string = "New waiipoint: x{} y{} z{}".format(wp_x, wp_y, wp_z)
			add_waypoint_maps(wp_x, wp_y, drone_id, 'WP')


	###################  RETURN  ######################
	return return_string


# receive GPS coordinates from offboard script (v2)
@app.route('/drone/receive_state', methods=['POST'])
def drone_receive_state():

	# get global variables 
	global bool_drone1_online, bool_drone2_online, bool_drone3_online
	global tri_dataset_temp


	###################  DECODE PAYLOAD  ######################
	print("!!!!!!!!! Data received from drone !!!!!!!!!")
	
	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the JSON received
	print("JSON received:")
	print(j)

	# parse received data
	r_pos_x     = float(j['x'])
	r_pos_y     = float(j['y'])
	r_pos_z     = float(j['z'])
	r_lat       = float(j['lat'])
	r_lng       = float(j['lng'])
	r_alt       = float(j['alt'])
	r_payload   = str(j['str'])
	r_ts_temp   = float(j['ts'])
	r_time      = dt.datetime.utcfromtimestamp(r_ts_temp).strftime(TIME_FORMAT)
	r_drone_id  = int(j['id'])
	r_nb_drone  = int(j['nb'])

	# type conversion and stuff
	r_timestamp = dt.datetime.utcfromtimestamp(r_ts_temp)
	r_time 		= str(r_time) 


	###################  CHANGE DRONE STATE  ######################
	# drone set to offline, greying button in gmaps
	if r_drone_id==1:
		bool_drone1_online = False
	elif r_drone_id==2:
		bool_drone2_online = False
	elif r_drone_id==3:
		bool_drone3_online = False


	###################  CREATE MEMORY  ######################
	
	# temporary estimate (data)
	if r_payload=='data' or r_payload=='data2':

		# match LoRa and drone (if possible)
		tri_datapoint = match_tri_datapoint(r_timestamp, r_pos_x, r_pos_y, r_pos_z, r_drone_id, r_payload)

		# add it if not None
		if tri_datapoint is not None:
			tri_dataset_temp.append(tri_datapoint)

	# normal datapoint
	else:
		# create new drone datapoint with parsed data
		datapoint = drone_datapoint()
		datapoint.pos_x 	= r_pos_x
		datapoint.pos_y 	= r_pos_y
		datapoint.pos_z 	= r_pos_z
		datapoint.lat       = r_lat
		datapoint.lng       = r_lng
		datapoint.alt       = r_alt
		datapoint.time 		= r_time
		datapoint.timestamp = r_timestamp
		datapoint.payload   = r_payload
		datapoint.drone_id  = r_drone_id

		# state
		if r_drone_id==1:
			datapoint.state = current_state1
		elif r_drone_id==2:
			datapoint.state = current_state2
		elif r_drone_id==3:
			datapoint.state = current_state3

		# save it in dataset
		drone_dataset.append(datapoint)


	###################  GET RETURN STRING  ######################

	# get return string with new instructions	
	return_string = get_return_string(r_payload, r_drone_id, r_nb_drone, r_pos_x, r_pos_y)


	# delay to emulate 5G
	#time.sleep(4)

	# success
	return return_string



#########################################################################################
################################### DATABASE CLEAR  #####################################
#########################################################################################

# deletes the database
@app.route('/delete_all')
def db_delete():
	#LoRa_datapoint.objects.delete()
	#drone_datapoint.objects.delete()
	#return 'Databases are now empty'
	return 'Delete function removed for (obvious) security reasons'



#########################################################################################
#####################################  APP START  #######################################
#########################################################################################

# start the app
if __name__ == '__main__':
	#app.debug = True
	app.run(host='0.0.0.0', port=port)