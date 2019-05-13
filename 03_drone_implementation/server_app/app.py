# misc dependencies
import os
import json
import struct
import math
import random
import numpy as np
import datetime as dt

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
cred = credentials.Certificate('drone-3bd2a-firebase-adminsdk-6ju7o-272f41c754.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://drone-3bd2a.firebaseio.com/'
})


#########################################################################################
#################################  CLASSES AND DATASETS  ################################
#########################################################################################

# class for LoRa messages
class LoRa_datapoint(db.Document):
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
	real_dist 	 = db.IntField()

# class and dataset for drone messages 
drone_dataset = []
class drone_datapoint:
	pos_x 		 = 666
	pos_y 		 = 666
	pos_z 		 = 666
	timestamp 	 = 666
	time 		 = ''
	payload 	 = ''
	state  		 = 666
	drone_id     = 1

# matrix for trilateration data	 
tri_dataset = []
class tri_datapoint:
	pos_x 		= 666
	pos_y 		= 666
	pos_z 		= 666
	esp 		= 666
	rssi 		= 666
	distance 	= 666

# to store the solution
class solution_datapoint:
	pos_x 		= 0
	pos_y 		= 0
	pos_z 		= 0
solution = solution_datapoint()

# to store the home
class home_datapoint:
	latitude    = 47.397751 	# Zurich
	longitude   = 8.545607 		# Zurich
	altitude    = 500 			# Zurich
	delta_x     = 0				# home is at zero
	delta_y     = 0				# home is at zero		
	delta_z     = 0				# home is at zero
home = home_datapoint()



#########################################################################################
###################################  LORA PARAMETERS  ###################################
#########################################################################################

# devices
gateway_printer   = '004A0DB4'
gateway_corner    = '004A1092'
gateway_drone     = '004A10EB'
device_eui        = '78AF580300000493'

# time format
TIME_FORMAT       = "%Y-%m-%dT%H:%M:%S.%f+02:00"		# change that to +01:00 in winter time
TIME_FORMAT_QUERY = "%Y-%m-%dT%H:%M:%S"



#########################################################################################
##################################  DRONE PARAMETERS  ###################################
#########################################################################################

# waypoint parameters
flying_altitude   = 20
takeoff_altitude  = 5
landing_radius    = 2    # circle around landing positions
hover_time 		  = 4

# storage for state
current_state 	  = 666
current_state1 	  = 666
current_state2 	  = 666
current_state3 	  = 666
nb_est_made       = 666

# for 3 drone: are the drones ready for next step
bool_drone1_ready = False
bool_drone2_ready = False
bool_drone3_ready = False

# for one or three drones: should the drone(s) start the algo (start from server)
bool_drone1_start = False
bool_drone2_start = False
bool_drone3_start = False

# kill switch
bool_kill_switch  = False



#########################################################################################
###############################  LOCALIZATION PARAMETERS  ###############################
#########################################################################################

# localization parameters
loop_todo		  = 1

# network position
network_x 		  = 0 
network_y 		  = 0	 	# used parameters (will be changed)
network_z 		  = 0

# radius of waypoints around est
circle_radius     = 666     # used parameters (will be changed)
circle_radius_v1  = 40 	    # base parameter when starting the simulation
circle_radius_v2  = 20	    # second parameter for second loop

# uncertainties radius
est_uncertainty1  = 150
est_uncertainty2  = 25
est_uncertainty3  = 10



#########################################################################################
##################################  GMAPS API THINGS  ###################################
#########################################################################################

# store position in Firebase
@app.route('/firebase/store_GPS', methods=['POST'])
def store_current_coord():

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
	lat = float(j['latitude'])
	lng = float(j['longitude'])
	alt = float(j['altitude'])
	drone_id = int(j['drone_id'])
	state = str(j['state'])
	r_timestamp = dt.datetime.utcfromtimestamp(float(j['timestamp']))

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
	    'drone_id': drone_id,
	    'state': state
	})

	# return string
	return 'Data added to Firebase'


# store home position in Firebase
@app.route('/firebase/store_home', methods=['POST'])
def store_current_home():

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
	r_lat     = float(j['latitude'])
	r_lng     = float(j['longitude'])
	r_alt     = float(j['altitude'])
	r_dx      = float(j['delta_x'])
	r_dy      = float(j['delta_y'])
	r_dz      = float(j['delta_z'])
	r_timestamp = dt.datetime.utcfromtimestamp(float(j['timestamp']))

	# add in memory
	global home
	home.latitude = r_lat
	home.longitude = r_lng
	home.altitude = r_alt
	home.delta_x = r_dx
	home.delta_y = r_dy
	home.delta_z = r_dz

	# push on Firebase
	print("Pushing home position on Firebase: lat={}, lng={}, alt={}".format(r_lat, r_lng, r_alt))
	ref_drone = firebase_db.reference('home')
	ref_drone.push({
		'lat': r_lat,
		'lng': r_lng,
		'altitude': r_alt,
		'delta_x': r_dx,
		'delta_y': r_dy,
		'delta_z': r_dz,
	    'sender': 'app.py: store_home',
	    'timestamp': (r_timestamp - dt.datetime(1970,1,1)).total_seconds(),
	})

	# return string
	return 'Data added to Firebase'


# empty Firebase when starting new run
@app.route('/firebase/empty', methods=['POST'])
def empty_firebase():

	print("!!!!!!!!! Deleting Firebase !!!!!!!!!")

	# delete database entries
	ref_drone  = firebase_db.reference('drone')
	ref_droneR = firebase_db.reference('droneR')
	ref_droneG = firebase_db.reference('droneG')
	ref_droneB = firebase_db.reference('droneB')
	ref_home   = firebase_db.reference('home')
	ref_netw   = firebase_db.reference('network')
	ref_est    = firebase_db.reference('estimate')
	ref_wayp   = firebase_db.reference('waypoint')
	ref_waypR  = firebase_db.reference('waypointR')
	ref_waypG  = firebase_db.reference('waypointG')
	ref_waypB  = firebase_db.reference('waypointB')
	ref_drone.delete()
	ref_droneR.delete()
	ref_droneG.delete()
	ref_droneB.delete()
	ref_home.delete()
	ref_netw.delete()
	ref_est.delete()
	ref_wayp.delete()
	ref_waypR.delete()
	ref_waypG.delete()
	ref_waypB.delete()

	return 'Success'


# add estimation to maps
def add_estimation_maps(pos_x, pos_y, radius):

	# convert in latlng
	lat, lng = conversion_xy_latlng(pos_x, pos_y)

	# push on Firebase
	ref_est = firebase_db.reference('estimate')
	ref_est.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'app.py: add_estimation',
	    'radius': radius
	})

	return 'Success'


# add waypoint on map
def add_waypoint_maps(pos_x, pos_y, drone_id):

	# convert in latlng
	lat, lng = conversion_xy_latlng(pos_x, pos_y)

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
	    'drone_id': drone_id
	})

	return 'Success'


# change network estimation
def add_network_maps(pos_x, pos_y):

	# convert in latlng
	lat, lng = conversion_xy_latlng(pos_x, pos_y)

	# push on Firebase
	ref_network = firebase_db.reference('network')
	ref_network.push({
		'lat': lat,
		'lng': lng,
	    'sender': 'app.py: add_network',
	})

	return 'Success'



#########################################################################################
###################################  TRILATERATION  #####################################
#########################################################################################

# converts the signal data to a distance
def function_signal_to_distance(esp, rssi):

	# exponential coefficents
	a_ESP  = 0.2011
	b_ESP  = -0.0556
	a_RSSI = 0.0273
	b_RSSI = -0.0777

	# compute based on ESP or RSSI
	distance = a_ESP * math.exp(b_ESP * esp)
	#distance = a_RSSI * math.exp(b_RSSI * rssi)

	# return the distance
	return distance


# uses the multilateration package on the created dataset
def trilateration(tri_dataset):
	# this function uses the localization package by Kamal Shadi:
	# https://github.com/kamalshadi/Localization

	# print
	print("LOC: Starting multilateration with {} anchors".format(len(tri_dataset)))

	# create new localization project using CCA method (centroid method) or LSE (least square error)
	P = lx.Project(mode='2D', solver='LSE')

	# create anchors and distances
	for i in range(0, len(tri_dataset)):
		P.add_anchor('anchor_n{}'.format(i), (tri_dataset[i].pos_x, tri_dataset[i].pos_y))
	t, label = P.add_target()
	for i in range(0, len(tri_dataset)):
		t.add_measure('anchor_n{}'.format(i), tri_dataset[i].distance)

	# solve the problem
	P.solve()
	print("LOC: Found solution at x={}, y={}, z={}".format(t.loc.x, t.loc.y, t.loc.z))
	return t.loc.x, t.loc.y, t.loc.z


# matches LoRa and drone data
def trilateration_main(drone_dataset):

	# clear tri_dataset
	global tri_dataset
	tri_dataset = []

	# matches LoRa and drone datasets
	for i in range(0, len(drone_dataset)):

		# ignore unrelevant states
		if drone_dataset[i].state not in {1,2,3}:
			continue

		# find matching LoRa points on interval
		timestamp = (drone_dataset[i].timestamp - dt.datetime(1970,1,1)).total_seconds() + 7200
		start = dt.datetime.fromtimestamp(timestamp-3)
		end = dt.datetime.fromtimestamp(timestamp+3)
		lora_data_in_interval = LoRa_datapoint.objects(timestamp__lt=end,timestamp__gt=start).first()
		if lora_data_in_interval is None:
			print("ERROR: no matching LoRa datapoint for timestamp {}".format(timestamp))
			continue
		else:
			# create tri dataset
			datapoint = tri_datapoint()
			datapoint.pos_x = float(drone_dataset[i].pos_x)
			datapoint.pos_y = float(drone_dataset[i].pos_y)
			datapoint.pos_z = float(drone_dataset[i].pos_z)

			# get distance estimate
			datapoint.esp  = lora_data_in_interval.gateway_esp[0]
			datapoint.rssi = lora_data_in_interval.gateway_rssi[0]
			datapoint.distance = float(function_signal_to_distance(datapoint.esp, datapoint.rssi))

			# save datapoint
			tri_dataset.append(datapoint)

	# test if empty (forgot to start LoRa node?)
	if len(tri_dataset) == 0:
		print("ERROR: empty trilateration dataset (didn't start LoRa node?)")
		return 0, 0, 0

	# test if enough points
	elif len(tri_dataset) < 3:
		print("ERROR: not enough datapoint for trilateration")
		return 0, 0, 0

	# all good
	else:
		# call trilateration function
		pos_x_est, pos_y_est, pos_z_est = trilateration(tri_dataset)

		# return result
		return pos_x_est, pos_y_est, pos_z_est



#########################################################################################
###################################  DEFAULT ROUTE  #####################################
#########################################################################################

# base route which just returns a string
@app.route('/')
def hello_world():
	return '<b>Welcome to the Drone application!</b>'



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
	network_x = j['pos_x']
	network_y = j['pos_y']
	network_z = j['pos_z']

	# add on map
	add_network_maps(network_x, network_y)

	# set network estimate
	global circle_radius_v1, circle_radius_v2
	circle_radius_v1 = j['radius_v1']
	circle_radius_v2 = j['radius_v2']

	# set hovering time
	global hover_time
	hover_time = j['hover_time']

	# set altitudes
	global flying_altitude, takeoff_altitude
	flying_altitude = j['flight']
	takeoff_altitude = j['takeoff']

	# set number of loops
	global loop_todo
	loop_todo = j['loop_todo']

	return "All parameters changed"


# batch print all parameters
@app.route('/param/print', methods=['GET'])
def param_print():

	return "<b>Parameters are:</b>\r\nNetwork position: x={}, y={}, z={}\r\nCircle radius: 1st={}, 2nd={}\r\nAltitudes: flying={}, takeoff={}\r\nAlgorithm loops to do: {}\r\nDrone status: {} {} {}\r\nSolution: x={}, y={}, z={}".format(network_x, network_y, network_z, circle_radius_v1, circle_radius_v2, flying_altitude, takeoff_altitude, loop_todo, bool_drone1_ready, bool_drone2_ready, bool_drone3_ready, solution.pos_x, solution.pos_y, solution.pos_z)


# to set the hovering time
@app.route('/drone/hover_time', methods=['POST'])
def drone_hover_time():
	print("!!!!!!!!! Hover time received from POST !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the coordinates received
	print("Hovering time received: h={}".format(j['hover_time']))

	# set hovering time
	global hover_time
	hover_time = j['hover_time']

	# success
	return 'Hovering time set at {}'.format(j['hover_time'])


# to set the takeoff and flight altitudes
@app.route('/drone/altitudes', methods=['POST'])
def drone_altitudes():
	print("!!!!!!!!! Hover time received from POST !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the param received
	print("Altitude received: takeoff={} and flight={}".format(j['takeoff'], j['flight']))

	# set altitudes
	global flying_altitude, takeoff_altitude
	flying_altitude = j['flight']
	takeoff_altitude = j['takeoff']

	# success
	return 'Altitudes set at takeoff={} and flight={}'.format(j['takeoff'], j['flight'])


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

	# add on map
	add_network_maps(j['pos_x'], j['pos_y'])

	# set network estimate
	global network_x, network_y, network_z
	network_x = j['pos_x']
	network_y = j['pos_y']
	network_z = j['pos_z']

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
	x, y = conversion_latlng_xy(float(j['lat']), float(j['lng']))
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


# to set the circle radii
@app.route('/lora/circle_radius', methods=['POST'])
def lora_circle_rad():
	print("!!!!!!!!! Circle radii received from POST !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display in log the param received
	print("Radii received: r1={}, r2={}".format(j['radius_v1'], j['radius_v2']))

	# set circle radii
	global circle_radius_v1, circle_radius_v2
	circle_radius_v1 = j['radius_v1']
	circle_radius_v2 = j['radius_v2']

	# success
	return 'Circle radii set at {} and {}'.format(j['radius_v1'], j['radius_v2'])



#########################################################################################
#####################################  DRONE STATUS  ####################################
#########################################################################################

# to set the drone ready manually for testing
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
	bool_drone1_ready = j['bool_drone1_ready']
	bool_drone2_ready = j['bool_drone2_ready']
	bool_drone3_ready = j['bool_drone3_ready']

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
	print("Drone start received: d1={}, d2={}, d3={} (kill={})".format(j['bool_drone1_start'], j['bool_drone3_start'], j['bool_drone3_start'], j['kill']))

	# set drone status
	global bool_kill_switch, bool_drone1_start, bool_drone2_start, bool_drone3_start
	bool_kill_switch  = j['kill']
	bool_drone1_start = j['bool_drone1_start']
	bool_drone2_start = j['bool_drone2_start']
	bool_drone3_start = j['bool_drone3_start']

	# return string
	if bool_kill_switch:
		return 'Set to kill the drones when possible!'
	else:
		return 'Drones starting modes set'


# GET method called from the drone to know if it should start
@app.route('/param/drone_ready_to_takeoff', methods=['POST'])
def param_drone_for_takeoff():
	print("!!!!!!!!! Drone is checking takeoff and kill switch !!!!!!!!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("ERROR: file is not a JSON")
		return 'Can only receive JSON file'

	# display info 
	drone_id = int(j['drone_id'])
	print("Asking for drone {}, params are d1={}, d2={}, d3={}, kill is {}".format(drone_id, bool_drone1_start, bool_drone2_start, bool_drone3_start, bool_kill_switch))

	# kill switch is active, kill drone
	if bool_kill_switch:
		return 'Kill drone now'

	# for each drone, return Y(es) or (N)o
	if drone_id == 1:
		if bool_drone1_start:
			return 'Y: drone {} ready for takeoff'.format(j['drone_id'])
		else:
			return 'N: drone {} not ready for takeoff'.format(j['drone_id'])
	elif drone_id == 2:
		if bool_drone2_start:
			return 'Y: drone {} ready for takeoff'.format(drone_id)
		else:
			return 'N: drone {} not ready for takeoff'.format(drone_id)
	elif drone_id == 3:
		if bool_drone3_start:
			return 'Y: drone {} ready for takeoff'.format(drone_id)
		else:
			return 'N: drone {} not ready for takeoff'.format(drone_id)
	else:
		print('ERROR: drone number unknown ({})'.format(j['drone_id']))
		return 'E: drone number unknown ({})'.format(j['drone_id'])
	


#########################################################################################
####################################  MISC FUNCTIONS  ###################################
#########################################################################################

# conversion between latlng and xy
def conversion_latlng_xy(lat, lng):

	# get home data
	home_lat = float(home.latitude)
	home_lng = float(home.longitude)		# home coordinates
	dx   	 = float(home.delta_x)
	dy 	 	 = float(home.delta_y)			# as home is not zero

	# math stuff for bearing 
	delta_lon = home_lng - lng
	bearing_y = math.sin(delta_lon) * math.cos(home_lat)
	bearing_x = math.cos(lat) * math.sin(home_lat) - math.sin(lat) * math.cos(home_lat) * math.cos(delta_lon)
	bearing = math.atan2(bearing_y, bearing_x)

	# distance geopy
	dist = geopy.distance.vincenty((lat, lng), (home_lat, home_lng)).m

	# return result
	return math.sin(bearing)*dist+dx, -math.cos(bearing)*dist+dy


# conversion between xy and latlng
def conversion_xy_latlng(x, y):
	
	# home to zero
	home_latlng = geopy.Point(home.latitude, home.longitude)
	dist_to_zero, bearing_to_zero = get_dist_bearing(-float(home.delta_x), -float(home.delta_y))
	zero_latlng = VincentyDistance(meters=dist_to_zero).destination(home_latlng, bearing_to_zero)

	# zero to drone
	distance, bearing = get_dist_bearing(x, y)
	latlng = VincentyDistance(meters=distance).destination(zero_latlng, bearing)

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
		bearing = math.degrees(math.atan(float(dist_y)/dist_x))
	
	# negative x modifier
	if dist_x < 0:
		bearing = -bearing-90
	else:
		bearing = -bearing+90

	# return
	return distance, bearing



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

	# parse communication parameters
	r_deveui    = j['DevEUI_uplink']['DevEUI']
	r_time      = j['DevEUI_uplink']['Time']
	r_timestamp = dt.datetime.strptime(r_time, TIME_FORMAT)
	r_sp_fact   = j['DevEUI_uplink']['SpFact']
	r_channel   = j['DevEUI_uplink']['Channel']
	r_band      = j['DevEUI_uplink']['SubBand']
	r_devtype   = "tuino-v3"
	r_tx_power  = j['DevEUI_uplink']['TxPower']

	# decode real calibration distance from payload info: possible 10,20,50,100,150,200, if not in calibration 255
	r_payload     = j['DevEUI_uplink']['payload_hex']
	r_payload_int = int(r_payload, 16)	# 16 as payload info in hexadecimal
	r_real_dist   = r_payload_int         # for now just distance in payload

	# store only one gateway information or all gateways
	store_only_one = True
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
	datapoint = LoRa_datapoint(devEUI=r_deveui, time=r_time, timestamp=r_timestamp, deviceType=r_devtype, sp_fact=r_sp_fact, 
		channel=r_channel, sub_band=r_band, gateway_id=g_id, gateway_rssi=g_rssi, gateway_snr=g_snr, 
		gateway_esp=g_esp, real_dist=r_real_dist, tx_power=r_tx_power)

	# save it to database
	datapoint.save()
	print('SUCESS: new LoRa datapoint saved to database')

	# success
	return 'Datapoint DevEUI %s saved' %(r_deveui)


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


# returns the waypoint based on drone id, total number of drone and state
def get_waypoint(drone_id, nb_drone, drone_dataset):

	# global variables
	global solution, circle_radius, nb_est_made

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
			# east of estimate
			wp_x = solution.pos_x + circle_radius*math.cos(0)
			wp_y = solution.pos_y + circle_radius*math.sin(0)
			wp_z = flying_altitude
		if state==1 or state==4:
			# north-west of estimate
			wp_x = solution.pos_x + circle_radius*math.cos(2*math.pi/3)
			wp_y = solution.pos_y + circle_radius*math.sin(2*math.pi/3)
			wp_z = flying_altitude
		if state==2 or state==5:
			# south-west of estimate
			wp_x = solution.pos_x + circle_radius*math.cos(4*math.pi/3)
			wp_y = solution.pos_y + circle_radius*math.sin(4*math.pi/3)
			wp_z = flying_altitude

		# need multilateration
		if state==3 or state==6:
			# do multilateration and store position
			pos_x_est, pos_y_est, pos_z_est = trilateration_main(drone_dataset)
			nb_est_made = nb_est_made + 1
			if pos_x_est == 0 and pos_y_est == 0 and pos_z_est == 0:
				print("LOC: Reusing previous estimate")
				solution.pos_x = solution.pos_x
				solution.pos_y = solution.pos_y		# reusing old estimation
				solution.pos_z = solution.pos_z
			else:
				solution.pos_x = pos_x_est
				solution.pos_y = pos_y_est 		# new calculated position
				solution.pos_z = pos_z_est
			
			# add estimate to map
			if state==3:
				add_estimation_maps(solution.pos_x, solution.pos_y, est_uncertainty2)
			if state==6:
				add_estimation_maps(solution.pos_x, solution.pos_y, est_uncertainty3)	

			# check if end is now
			if nb_est_made==loop_todo:
				# finish program, return landing waypoint above estimate
				wp_x = solution.pos_x
				wp_y = solution.pos_y
				wp_z = flying_altitude
				bool_landing_waypoint = True
			else:
				# program still running, new parameters
				circle_radius = circle_radius_v2

				# east of (new) estimate
				wp_x = solution.pos_x + circle_radius*math.cos(0)
				wp_y = solution.pos_y + circle_radius*math.sin(0)
				wp_z = flying_altitude

		# too high state
		if state > 6:
			print("ERROR: too high state")
			wp_x = 0
			wp_y = 0
			wp_z = flying_altitude

	# three drones
	if nb_drone==3:

		# if unknown drone number
		if drone_id>3 or drone_id<1:
			print("ERROR: unknown drone number (not 1/2/3)")
			wp_x = 0
			wp_y = 0
			wp_z = flying_altitude

		# first waypoints
		if state==0:
			if drone_id==1:
				# east of estimate
				wp_x = solution.pos_x + circle_radius*math.cos(0)
				wp_y = solution.pos_y + circle_radius*math.sin(0)
				wp_z = flying_altitude
			elif drone_id==2:
				# north-west of estimate
				wp_x = solution.pos_x + circle_radius*math.cos(2*math.pi/3)
				wp_y = solution.pos_y + circle_radius*math.sin(2*math.pi/3)
				wp_z = flying_altitude
			elif drone_id==3:
				# south-west of estimate
				wp_x = solution.pos_x + circle_radius*math.cos(4*math.pi/3)
				wp_y = solution.pos_y + circle_radius*math.sin(4*math.pi/3)
				wp_z = flying_altitude

		# estimations
		if state>0:
			# do multilateration and store position
			pos_x_est, pos_y_est, pos_z_est = trilateration_main(drone_dataset)
			nb_est_made = state
			if pos_x_est == 0 and pos_y_est == 0 and pos_z_est == 0:
				print("LOC: Reusing previous estimate")
				solution.pos_x = solution.pos_x
				solution.pos_y = solution.pos_y		# reusing old estimation
				solution.pos_z = solution.pos_z
			else:
				solution.pos_x = pos_x_est
				solution.pos_y = pos_y_est 		# new calculated position
				solution.pos_z = pos_z_est

			# add estimate to map
			add_estimation_maps(solution.pos_x, solution.pos_y, est_uncertainty2)

			# check if end is now
			if nb_est_made==loop_todo:
				# finish program, return landing waypoint above estimate
				if drone_id==1:
					wp_x = solution.pos_x + landing_radius*math.cos(0)
					wp_y = solution.pos_y + landing_radius*math.sin(0)
					wp_z = flying_altitude
				elif drone_id==2:
					wp_x = solution.pos_x + landing_radius*math.cos(2*math.pi/3)
					wp_y = solution.pos_y + landing_radius*math.sin(2*math.pi/3)
					wp_z = flying_altitude
				elif drone_id==3:
					wp_x = solution.pos_x + landing_radius*math.cos(4*math.pi/3)
					wp_y = solution.pos_y + landing_radius*math.sin(4*math.pi/3)
					wp_z = flying_altitude
				bool_landing_waypoint = True
			else:
				# program still running, new parameters
				circle_radius = circle_radius_v2

				# new waypoint
				if drone_id==1:
					wp_x = solution.pos_x + circle_radius*math.cos(0)
					wp_y = solution.pos_y + circle_radius*math.sin(0)
					wp_z = flying_altitude
					bool_drone1_ready = False
				elif drone_id==2:
					wp_x = solution.pos_x + circle_radius*math.cos(2*math.pi/3)
					wp_y = solution.pos_y + circle_radius*math.sin(2*math.pi/3)
					wp_z = flying_altitude
					bool_drone2_ready = False
				elif drone_id==3:
					wp_x = solution.pos_x + circle_radius*math.cos(4*math.pi/3)
					wp_y = solution.pos_y + circle_radius*math.sin(4*math.pi/3)
					wp_z = flying_altitude
					bool_drone3_ready = False

	# save on map
	add_waypoint_maps(wp_x, wp_y, drone_id)

	return wp_x, wp_y, wp_z, bool_landing_waypoint


# receive GPS coordinates from offboard script (v2)
@app.route('/drone/receive_state', methods=['POST'])
def drone_receive_state():
	# get global variables 
	global drone_dataset, current_state1, current_state2, current_state3, solution
	global network_x, network_y, network_z, circle_radius, nb_est_made
	global bool_drone1_ready, bool_drone2_ready, bool_drone3_ready


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
	r_pos_x     = float(j['pos_x'])
	r_pos_y     = float(j['pos_y'])
	r_pos_z     = float(j['pos_z'])
	r_payload   = j['payload']
	r_ts_temp   = float(j['timestamp'])
	r_time      = dt.datetime.utcfromtimestamp(r_ts_temp).strftime(TIME_FORMAT)
	r_drone_id  = int(j['drone_id'])
	r_nb_drone  = int(j['nb_drone'])

	# type conversion and stuff
	r_timestamp = dt.datetime.utcfromtimestamp(r_ts_temp)
	r_time 		= str(r_time) 


	###################  SWITCH STATE  ######################
	
	# if it is not set during the next if's
	return_string = 'ERROR, return string not set'

	# switch to OFFBOARD mode done
	if r_payload=='drone_offboard':
		return_string = "Congrats on offboard mode!"

	# drone was just armed
	if r_payload=='drone_armed':

		# add waypoint for takeoff to Firebase
		add_waypoint_maps(r_pos_x, r_pos_y, r_drone_id)

		# return string with takeoff coordinates
		return_string = "Takeoff at current position: h{}".format(takeoff_altitude)

	# drone took off and is in the air
	if r_payload=='drone_takeoff':

		# empty drone dataset
		print("Emptying drone dataset for this mission")
		drone_dataset = []

		# set base solution at network estimate
		solution.pos_x = network_x
		solution.pos_y = network_y
		solution.pos_z = 0

		# set base parameters
		circle_radius = circle_radius_v1

		# set state at 0
		if r_drone_id==1:
			current_state1 = 0
		if r_drone_id==2:
			current_state2 = 0
		if r_drone_id==3:
			current_state3 = 0

		## all drones needs to be ready again
		bool_drone1_ready = False
		bool_drone2_ready = False
		bool_drone3_ready = False

		# set estimation made at 0
		nb_est_made = 0

		# get waypoint
		wp_x, wp_y, wp_z, bool_landing_waypoint = get_waypoint(r_drone_id, r_nb_drone, drone_dataset)

		# save on map
		add_network_maps(network_x, network_y)		# point
		add_estimation_maps(solution.pos_x, solution.pos_y, est_uncertainty1) 	# circle

		# return string with new coordinates
		return_string = "New waypoint: x{} y{} z{}".format(wp_x, wp_y, wp_z)

	# drone reached its previous (unknown) waypoint
	if r_payload=='waypoint_reached':

		# read old state to increment it
		if r_drone_id==1:
			current_state1 = current_state1 + 1
		if r_drone_id==2:
			current_state2 = current_state2 + 1
		if r_drone_id==3:
			current_state3 = current_state3 + 1

		# send hover time
		return_string = "Hover time set: h{}".format(hover_time)

	# drone is hovering in position
	if r_payload=='data_collected':

		# return string
		return_string = "Wait at same position until end of hovering"

	# drone has finished hovering
	if r_payload=='waiting_for_command':

		# bool if ready for next waypoint
		bool_ready_for_next_waypoint = False

		# fill bool_ready
		if r_drone_id==1:
			bool_drone1_ready = True
		if r_drone_id==2:
			bool_drone2_ready = True
		if r_drone_id==3:
			bool_drone3_ready = True

		# for one drone, true
		if r_nb_drone==1:
			bool_ready_for_next_waypoint = True

		# for three drones: check if all drones have finished their hovering
		if r_nb_drone==3:
			if bool_drone1_ready == True and bool_drone2_ready == True and bool_drone3_ready == True:
				bool_ready_for_next_waypoint = True

		if bool_ready_for_next_waypoint:
			# get new waypoint
			wp_x, wp_y, wp_z, bool_landing_waypoint = get_waypoint(r_drone_id, r_nb_drone, drone_dataset)

			# return string
			if bool_landing_waypoint:
				return_string = "Land at position: x{} y{} z{}".format(wp_x, wp_y, wp_z)
			else:
				return_string = "New waypoint: x{} y{} z{}".format(wp_x, wp_y, wp_z)
		else:
			# still waiting
			return_string = "Waiting until other drones ready"

	# drone is landing
	if r_payload=='drone_landing':
		return_string = "Congrats on mission!"


	###################  CREATE MEMORY  ######################
	# create new datapoint with parsed data
	datapoint = drone_datapoint()
	datapoint.pos_x 	= r_pos_x
	datapoint.pos_y 	= r_pos_y
	datapoint.pos_z 	= r_pos_z
	datapoint.time 		= r_time
	datapoint.timestamp = r_timestamp
	datapoint.payload   = r_payload
	datapoint.state 	= current_state
	datapoint.drone_id  = r_drone_id

	# save it
	drone_dataset.append(datapoint)

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