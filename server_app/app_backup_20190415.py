# import dependencies
import os
import json
import struct
import math
import numpy as np
import datetime as dt
from flask import Flask, Response, request, redirect, url_for, escape, jsonify, make_response
from flask_mongoengine import MongoEngine
from itertools import chain



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


# create the two classes for LoRa and drone messages
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

class drone_datapoint(db.Document):
	pos_x 		 = db.FloatField()
	pos_y 		 = db.FloatField()
	pos_z 		 = db.FloatField()
	timestamp    = db.DateTimeField()
	time 		 = db.StringField()
	payload      = db.StringField()

# create the class for server memory
class server_memory(db.Document):
	state 		 = db.IntField()
	timestamp    = db.DateTimeField()
	time 		 = db.StringField()



# set the port dynamically with a default of 3000 for local development
port = int(os.getenv('PORT', '3000'))

# devices
gateway_printer = '004A0DB4'
gateway_corner  = '004A1092'
gateway_drone   = '004A10EB'
device_eui      = '78AF580300000493'

# time format
TIME_FORMAT       = "%Y-%m-%dT%H:%M:%S.%f+02:00"
TIME_FORMAT_QUERY = "%Y-%m-%dT%H:%M:%S"

# waypoint parameters
flying_altitude   = 10
takeoff_altitude  = 2
network_x 		  = 0
network_y 		  = 100
network_z 		  = 0
circle_radius     = 50
hover_time 		  = 2




#########################################################################################
###################################  TRILATERATION  #####################################
#########################################################################################

# trilateration functions
def trilateration():

	# dummy position for testing
	pos_x = -100
	pos_y = 0
	pos_z = takeoff_altitude

	return pos_x, pos_y, pos_z



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
	print("!!! Data received from ThingPark !!!")

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
			return 'Datapoint not saved (not received by set gateway)'
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
	print("!!! Data received from drone !!!")
	
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
	r_pos_x     = j['pos_x']
	r_pos_y     = j['pos_y']
	r_pos_z     = j['pos_z']
	r_payload   = j['payload']
	r_ts_temp   = j['timestamp']
	r_time      = dt.datetime.utcfromtimestamp(float(r_ts_temp)).strftime(TIME_FORMAT)

	# type conversion and stuff
	r_timestamp = dt.datetime.utcfromtimestamp(float(r_ts_temp))
	r_time 		= str(r_time) 

	# create new datapoint with parsed data
	datapoint = drone_datapoint(pos_x=r_pos_x, pos_y=r_pos_y, pos_z=r_pos_z, time=r_time, timestamp=r_timestamp, payload=r_payload)

	# save it to database
	datapoint.save()
	print('SUCESS: new drone datapoint saved to database')

	# process payload
	if r_payload=='drone_armed':
		# set takeoff position as 2m above current position
		pos_x = float(r_pos_x)
		pos_y = float(r_pos_y)
		pos_z = float(r_pos_z) + float(takeoff_altitude)
		return "New waypoint (takeoff): x{} y{} z{}".format(pos_x, pos_y, pos_z)

	if r_payload=='drone_takeoff':
		# empty memory database
		server_memory.objects.delete()

		# set state
		current_state = 0
		print('STATE: current state is {}'.format(current_state))
		memory = server_memory(state=current_state)
		memory.save()

		# send first waypoint
		pos_x = network_x + circle_radius*math.cos(0)
		pos_y = network_y + circle_radius*math.sin(0)
		pos_z = flying_altitude
		return "New waypoint: x{} y{} z{}".format(pos_x, pos_y, pos_z)

	if r_payload=='waypoint_reached':
		# read old state to increment it
		latest = server_memory.objects.order_by('-id').first()
		current_state = latest.state + 1

		# set state
		print('STATE: current state is {}'.format(current_state))
		memory = server_memory(state=current_state, timestamp=r_timestamp, time=r_time)
		memory.save()

		# send hover time
		return "Hover time set: h{}".format(hover_time)

	if r_payload=='data_collected':
		# read old state to know which waypoint is next
		latest = server_memory.objects.order_by('-id').first()

		# create and send waypoint
		if latest.state == 1:
			pos_x = network_x + circle_radius*math.cos(2*math.pi/3)
			pos_y = network_y + circle_radius*math.sin(2*math.pi/3)
			pos_z = flying_altitude
		elif latest.state == 2:
			pos_x = network_x + circle_radius*math.cos(4*math.pi/3)
			pos_y = network_y + circle_radius*math.sin(4*math.pi/3)
			pos_z = flying_altitude
		else:
			pos_x, pos_y, pos_z = trilateration()
			#pos_x = network_x
			#pos_y = network_y
			#pos_z = flying_altitude
			return "Land at position: x{} y{} z{}".format(pos_x, pos_y, pos_z)
		return "New waypoint: x{} y{} z{}".format(pos_x, pos_y, pos_z)

	# success
	return 'Datapoint added to database'



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


#querying the database and giving back a JSON file
@app.route('/drone/query', methods=['GET'])
def drone_query():
	query = request.args

	# to delete datapoint based on time
	if 'delete_time_point' in query and 'time' in query:
		deltime_start = dt.datetime.strptime(query['time'], TIME_FORMAT_QUERY) - dt.timedelta(seconds=2)
		deltime_end = dt.datetime.strptime(query['time'], TIME_FORMAT_QUERY) + dt.timedelta(seconds=2)
		drone_datapoint.objects(timestamp__lt=deltime_end, timestamp__gt=deltime_start).delete()
		return 'point deleted'

	# to delete datapoints based on time
	if 'delete_time_interval' in query and 'start' in query and 'end' in query:
		end = dt.datetime.strptime(query['end'], TIME_FORMAT_QUERY)
		start = dt.datetime.strptime(query['start'], TIME_FORMAT_QUERY)
		drone_datapoint.objects(timestamp__lt=end,timestamp__gt=start).delete()
		return 'time interval deleted'

	# defaults: only in the previous year
	start = dt.datetime.now() - dt.timedelta(days=365)  # begins selection one year ago
	end = dt.datetime.now() + dt.timedelta(hours=2)		# just in case we have timestamps in the future?

	# change start and end time
	if 'start' in query:
		start = dt.datetime.strptime(query['start'], TIME_FORMAT_QUERY)
	if 'end' in query:
		end = dt.datetime.strptime(query['end'], TIME_FORMAT_QUERY)

	datapoints = drone_datapoint.objects(timestamp__lt=end,timestamp__gt=start).to_json()

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


# output JSON as downloaded file, drone database
@app.route('/drone/json', methods=['GET'])
def drone_export_json():
	print('Exporting drone database as JSON')
	return Response(drone_datapoint.objects.to_json(),mimetype='application/json',
		headers={'Content-Disposition':'attachment;filename=database.json'})


# print JSON directly in browser, drone database
@app.route('/drone/json_print', methods=['GET'])
def drone_print_json():
	print('Printing drone database as JSON')
	return drone_datapoint.objects.to_json()


# print JSON directly in browser, memory database
@app.route('/memory/json_print', methods=['GET'])
def memory_print_json():
	print('Printing memory database as JSON')
	return server_memory.objects.to_json()



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