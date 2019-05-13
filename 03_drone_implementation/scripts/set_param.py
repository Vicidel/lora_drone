import requests

pos_x=0
pos_y=100
pos_z=0
radius_v1=10
radius_v2=5
hover_time=4
flight=10
takeoff=2
loop_todo=1

r = requests.post("http://victor.scapp.io/param/change", json={'pos_x': pos_x, 'pos_y': pos_y, 'pos_z': pos_z, 'radius_v1': radius_v1, 'radius_v2': radius_v2, 'hover_time': hover_time, 'flight': flight, 'takeoff': takeoff, 'loop_todo': loop_todo})
print(r.status_code, r.reason)
print(r.text[:300] + '...')
