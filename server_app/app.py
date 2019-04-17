# import dependencies
import os
import json
import struct
import math
import numpy as np
import datetime as dt
import localization as lx
from flask import Flask, Response, request, redirect, url_for, escape, jsonify, make_response
from flask_mongoengine import MongoEngine
from itertools import chain





#########################################################################################
####################################  APP PARAMETERS  ###################################
#########################################################################################

# bootstrap the app
app = Flask(__name__)

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
	pos_x 		= 6.66
	pos_y 		= 6.66
	pos_z 		= 6.66



#########################################################################################
###################################  LORA PARAMETERS  ###################################
#########################################################################################

# devices
gateway_printer   = '004A0DB4'
gateway_corner    = '004A1092'
gateway_drone     = '004A10EB'
device_eui        = '78AF580300000493'

# time format
TIME_FORMAT       = "%Y-%m-%dT%H:%M:%S.%f+02:00"
TIME_FORMAT_QUERY = "%Y-%m-%dT%H:%M:%S"



#########################################################################################
##################################  DRONE PARAMETERS  ###################################
#########################################################################################

# waypoint parameters
flying_altitude   = 10
takeoff_altitude  = 2
network_x_v1 	  = 0	 # base parameter when starting the simulation
network_y_v1 	  = 0	 # base parameter when starting the simulation
network_z_v1 	  = 0	 # base parameter when starting the simulation
network_x 		  = 666	 # current parameter
network_y 		  = 666	 # current parameter
network_z 		  = 666	 # current parameter
circle_radius     = 666
circle_radius_v1  = 100	 # base parameter when starting the simulation
circle_radius_v2  = 40	 # second parameter for second loop
hover_time 		  = 10

# storage for state
current_state 	  = 666

# localization parameters
loop_todo		  = 2
solution 		  = solution_datapoint()



#########################################################################################
###################################  TRILATERATION  #####################################
#########################################################################################

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



#########################################################################################
####################################  DRONE ROUTE  ######################################
#########################################################################################

# receive GPS coordinates from offb_node script
@app.route('/drone/receive_message', methods=['POST'])
def drone_receive():
	# get global variables 
	global drone_dataset, current_state, solution
	global network_x, network_y, network_z, circle_radius

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
	r_pos_x     = j['pos_x']
	r_pos_y     = j['pos_y']
	r_pos_z     = j['pos_z']
	r_payload   = j['payload']
	r_ts_temp   = float(j['timestamp'])
	r_time      = dt.datetime.utcfromtimestamp(r_ts_temp).strftime(TIME_FORMAT)

	# type conversion and stuff
	r_timestamp = dt.datetime.utcfromtimestamp(r_ts_temp)
	r_time 		= str(r_time) 


	###################  PRINT STUFF  ######################
	print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
	print("DEBUG: solution", solution.pos_x, solution.pos_y, solution.pos_z)
	print("DEBUG: param", network_x, network_y, circle_radius)
	print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")


	###################  SWITCH STATE  ######################
	
	# if it is not set during the next if's
	return_string = 'ERROR, return string not set'

	# switch to OFFBOARD mode done
	if r_payload=='drone_offboard':
		return_string = "Congrats on offboard mode"

	# drone was just armed
	if r_payload=='drone_armed':

		# set takeoff position as 2m above current position
		pos_x = float(r_pos_x)
		pos_y = float(r_pos_y)
		pos_z = float(r_pos_z) + float(takeoff_altitude)

		# return string with takeoff coordinates
		return_string = "New waypoint (takeoff): x{} y{} z{}".format(pos_x, pos_y, pos_z)

	# drone took off and is in the air
	if r_payload=='drone_takeoff':

		# empty drone dataset
		print("Emptying drone dataset for this mission")
		drone_dataset = []

		# set base parameters
		current_state = 0
		network_x 	  = network_x_v1
		network_y 	  = network_y_v1
		network_z 	  = network_z_v1
		circle_radius = circle_radius_v1

		# first waypoint
		pos_x = network_x + circle_radius*math.cos(0)
		pos_y = network_y + circle_radius*math.sin(0)
		pos_z = flying_altitude

		# return string with new coordinates
		return_string = "New waypoint: x{} y{} z{}".format(pos_x, pos_y, pos_z)

	# drone reached its previous (unknown) waypoint
	if r_payload=='waypoint_reached':

		# read old state to increment it
		current_state = current_state + 1

		# send hover time
		return_string = "Hover time set: h{}".format(hover_time)

	# drone is hovering in position
	if r_payload=='data_collected':

		# create and send next waypoint
		if (current_state == 1) or (current_state == 4):
			pos_x = network_x + circle_radius*math.cos(2*math.pi/3)
			pos_y = network_y + circle_radius*math.sin(2*math.pi/3)
			pos_z = flying_altitude
			return_string = "New waypoint: x{} y{} z{}".format(pos_x, pos_y, pos_z)
		elif (current_state == 2) or (current_state == 5):
			pos_x = network_x + circle_radius*math.cos(4*math.pi/3)
			pos_y = network_y + circle_radius*math.sin(4*math.pi/3)
			pos_z = flying_altitude
			return_string = "New waypoint: x{} y{} z{}".format(pos_x, pos_y, pos_z)
		elif current_state == 3:
			pos_x_est, pos_y_est, pos_z_est = trilateration_main(drone_dataset)
			solution.pos_x = pos_x_est
			solution.pos_y = pos_y_est
			solution.pos_z = pos_z_est

			if loop_todo == 1:
				print("Go for landing at found position")
				return_string = "Land at position: x{} y{} z{}".format(pos_x, pos_y, pos_z)
			else:
				print("Restarting around estimation, smaller parameters")

				# new parameters
				network_x = pos_x_est
				network_y = pos_y_est
				network_z = pos_z_est
				circle_radius = circle_radius_v2

				# new waypoint
				pos_x = network_x + circle_radius*math.cos(0)
				pos_y = network_y + circle_radius*math.sin(0)
				pos_z = flying_altitude

				return_string = "New waypoint: x{} y{} z{}".format(pos_x, pos_y, pos_z)
		elif current_state == 6:
			pos_x, pos_y, pos_z = trilateration_main(drone_dataset)
			print("Go for landing at found position")
			return_string = "Land at position: x{} y{} z{}".format(pos_x, pos_y, pos_z)
		else:
			return_string = "ERROR, unknown state"

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

	# save it
	drone_dataset.append(datapoint)

	# success
	return return_string



#########################################################################################
######################################  QUERIES  ########################################
#########################################################################################

#querying the database and giving back a JSON file
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



#########################################################################################
####################################  JSON OUTPUT  ######################################
#########################################################################################

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