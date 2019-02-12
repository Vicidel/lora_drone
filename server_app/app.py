# import dependencies
import os
import json
import struct
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

class DataPoint(db.Document):
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

# set the port dynamically with a default of 3000 for local development
port = int(os.getenv('PORT', '3000'))

# devices
gateway_printer = '004A0DB4'
gateway_corener = '004A1092'
gateway_drone   = '004A10EB'
device_eui      = '78AF580300000493'



# base route which just returns a string
@app.route('/')
def hello_world():
	return '<b>Welcome to the Drone application!</b>'


# # adds a random datapoint to DB
# @app.route('/test')
# def test_function():
# 	# create dummy datapoint
# 	datapoint = DataPoint(devEUI='testestest', time='now', deviceType='testdevice', sp_fact=666, 
# 		channel='666', sub_band='666')
# 	print('adding test datapoint')
# 	datapoint.save()
# 	return 'Added testing datapoint with devEUI "testestest"'


# Swisscom LPN listener to POST from actility
@app.route('/sc_lpn', methods=['POST'])
def sc_lpn():
	print("!!! Data received from ThingPark !!!")

	# test nature of message: if not JSON we don't want it
	j = []
	try:
		j = request.json
	except:
		print("file is not a JSON: error")
		return 'Can only receive JSON file'

	# display in log the JSON received
	print("JSON received:")
	print(j)

	# parse communication parameters
	r_deveui    = j['DevEUI_uplink']['DevEUI']
	r_time      = j['DevEUI_uplink']['Time']
	r_timestamp = dt.datetime.strptime(r_time,"%Y-%m-%dT%H:%M:%S.%f+01:00")
	r_sp_fact   = j['DevEUI_uplink']['SpFact']
	r_channel   = j['DevEUI_uplink']['Channel']
	r_band      = j['DevEUI_uplink']['SubBand']
	r_devtype   = "tuino-v3"
	r_tx_power  = j['DevEUI_uplink']['TxPower']

	# decode real calibration distance from payload info: possible 10,20,50,100,150,200, if not in calibration 255
	r_payload     = j['DevEUI_uplink']['payload_hex']
	r_payload_int = int(r_payload, 16)	# 16 as payload info in hexadecimal
	r_real_dist   = r_payload_int         # for now just distance in payload

	# store only drone gateway information or all gateways
	store_only_drone = True
	if store_only_drone:
		# initialize g_ig
		g_id = []

		# store only metadata of drone gateway
		for item in j['DevEUI_uplink']['Lrrs']['Lrr']:
			if gateway_drone == item['Lrrid']:
				g_id   = item['Lrrid']
				g_rssi = item['LrrRSSI']
				g_snr  = item['LrrSNR']
				g_esp  = item['LrrESP']
		
		# if drone gateway not detected
		if not g_id:
			print('information not received by drone GW')
			return 'Datapoint not saved (not received by drone GW)'
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
	datapoint = DataPoint(devEUI=r_deveui, time= r_time, timestamp=r_timestamp, deviceType=r_devtype, sp_fact=r_sp_fact, 
		channel=r_channel, sub_band=r_band, gateway_id=g_id, gateway_rssi=g_rssi, gateway_snr=g_snr, 
		gateway_esp=g_esp, real_dist=r_real_dist, tx_power=r_tx_power)
	datapoint.save()
	print('new datapoint saved to database')

	# success
	return 'Datapoint DevEUI %s saved' %(r_deveui)


# output JSON
@app.route('/json', methods=['GET'])
def print_json():
	print('exporting full database as JSON')
	#return DataPoint.objects.to_json() 		# for directly in browser
	return Response(DataPoint.objects.to_json(),mimetype='application/json', 	# for automatic file download
		headers={'Content-Disposition':'attachment;filename=database.json'})


#querying the database and giving back a JSON file
@app.route('/query', methods=['GET'])
def db_query():
	query = request.args
 
	# defaults: only with 0 txpow, sf of 7 and in the previous year
	sf = 7
	start = dt.datetime.now() - dt.timedelta(days=365)  # begins selection one year ago
	end = dt.datetime.now() + dt.timedelta(hours=2)		# just in case we have timestamps in the future?

	#enable for deleting objects. Attention, deletes parts of the database! 
	if 'delete' in query and 'start' in query and 'end' in query:
		end = dt.datetime.strptime(query['end'], TIME_FORMAT)
		start = dt.datetime.strptime(query['start'], TIME_FORMAT)
		#DataPoint.objects(track_ID=query['delete'],timestamp__lt=end,timestamp__gt=start).delete()
		#return 'objects deleted'
		return 'delete feature disabled for security reasons'

	if 'delpoint' in query:
		print('query for deleting point received')
		deltime_start = dt.datetime.strptime(query['delpoint'], TIME_FORMAT_DEL) - dt.timedelta(seconds=2)
		deltime_end = dt.datetime.strptime(query['delpoint'], TIME_FORMAT_DEL) + dt.timedelta(seconds=2)
		n_points = DataPoint.objects(timestamp__lt=deltime_end, timestamp__gt=deltime_start).count()
		DataPoint.objects(timestamp__lt=deltime_end, timestamp__gt=deltime_start).delete()
		return '{} points deleted'.format(n_points)

	if 'start' in query:
		start = dt.datetime.strptime(query['start'], TIME_FORMAT)

	if 'end' in query:
		end = dt.datetime.strptime(query['end'], TIME_FORMAT)

	if 'sf' in query:
		sf = int(query['sf'])

	# for selection of only data corresponding to one end-device
	if 'device' in query:
		device = query['device']
		datapoints = DataPoint.objects(devEUI=device,timestamp__lt=end,timestamp__gt=start,sp_fact=sf).to_json()
	else:
		datapoints = DataPoint.objects(timestamp__lt=end,timestamp__gt=start,sp_fact=sf).to_json()

	#return datapoints 		# for directly in browser
	return Response(datapoints,mimetype='application/json', 	# for automatic file download
		headers={'Content-Disposition':'attachment;filename=query.json'})


# delete the database: be careful !! 
@app.route('/delete_all')
def delete_function():
	#Datapoint.objects.delete()
	#return 'database is now empty'
	return 'delete function removed for security reasons'



# start the app
if __name__ == '__main__':
	#app.debug = True
	app.run(host='0.0.0.0', port=port)