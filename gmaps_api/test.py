import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

import math
import datetime
import random 

def timestamp_now():
	now = datetime.datetime.utcnow()
	return (now - datetime.datetime(1970,1,1)).total_seconds()

# Fetch the service account key JSON file contents
cred = credentials.Certificate('drone-3bd2a-firebase-adminsdk-6ju7o-272f41c754.json')
# Initialize the app with a service account, granting admin privileges
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://drone-3bd2a.firebaseio.com/'
})

# empty at program start
db.reference('start').delete()
db.reference('drone').delete()

# create latlng
takeoff_spot_lat = 46.51587575161965
takeoff_spot_lng = 6.564940909970005
drone_lat = 46.515115818943954
drone_lng = 6.565898777771054

ref_start = db.reference('/start')
ref_start.push({
    'lat': takeoff_spot_lat,
    'lng': takeoff_spot_lng,
    'sender': 'test.py',
    'timestamp': timestamp_now()
})

ref_drone = db.reference('/drone')
ref_drone.push({
	'lat': drone_lat,
	'lng': drone_lng,
    'sender': 'test.py',
    'timestamp': timestamp_now()
})

input("df")
for i in range(0,10):
	drone_lat = float(random.uniform(4651, 4652)/100)
	drone_lng = float(random.uniform(656, 657)/100)
	ref_drone = db.reference('/drone')
	ref_drone.push({
		'lat': drone_lat,
		'lng': drone_lng,
	    'sender': 'test.py loop',
	    'timestamp': timestamp_now()
	})
	input('now')