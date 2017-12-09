import krpc
import os
import time
import numpy as np

conn = krpc.connect(name='Runway Recorder', address='localhost')
vessel = conn.space_center.active_vessel
kerbin = conn.space_center.bodies['Kerbin']

while vessel.control.sas == True:
	time.sleep(0.1)

position_history = []
while vessel.control.sas == False:
	os.system('clear')
	print vessel.position(kerbin.reference_frame)
	print np.linalg.norm(vessel.velocity(kerbin.reference_frame))
	position_history.append(vessel.position(kerbin.reference_frame))
	time.sleep(0.05)

import json
with open('ksc_runway.json', 'w') as f:
	json.dump(position_history, f)
