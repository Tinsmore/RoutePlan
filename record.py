import json
import os


replay_file = "examples/hangzhou_4x4_gudang_18041610_1h/replay.txt"
data_file = "examples/hangzhou_4x4_gudang_18041610_1h/road_data.json"

f = open(replay_file, "r")
records = f.readlines()
f.close()

if not os.path.exists(data_file):
    road_data = {}
else:
	f_data = open("examples/hangzhou_4x4_gudang_18041610_1h/road_data.json", 'r')
	road_data = json.load(f_data)
	f_data.close()

for record in records:
	record = record.split(";")[-1]
	roads = record.split(",")[:-1]
	for road in roads:
		road = road.split()
		rid = road[0]
		if rid not in road_data:
			road_data[rid] = {"left": 0, "straight": 0,
			 "l_sum": 0, "s_sum": 0, 
			 "l_cons": 0, "s_cons": 0, 
			 "l_ct": 0, "s_ct": 0,
			 "last_l": 0, "last_s": 0}

		if road[1] == 'r':
			left = 1
		else:
			left = 0
		if road[2] == 'r':
			straight = 1
		else:
			straight = 0

		if left<road_data[rid]["last_l"]:
			road_data[rid]["l_sum"] += road_data[rid]["l_cons"]
			road_data[rid]["l_cons"] = 0
			road_data[rid]["l_ct"] += 1
		else:
			road_data[rid]["l_cons"] += 1
		road_data[rid]["last_l"] = left

		if straight<road_data[rid]["last_s"]:
			road_data[rid]["s_sum"] += road_data[rid]["s_cons"]
			road_data[rid]["s_cons"] = 0
			road_data[rid]["s_ct"] += 1
		else:
			road_data[rid]["s_cons"] += 1
		road_data[rid]["last_s"] = straight

for rid in road_data:
	if road_data[rid]["l_ct"] > 0:
		road_data[rid]["left"] = round(1.0*road_data[rid]["l_sum"]/road_data[rid]["l_ct"], 2)
	if road_data[rid]["s_ct"] > 0:
		road_data[rid]["straight"] = round(1.0*road_data[rid]["s_sum"]/road_data[rid]["s_ct"], 2)
	road_data[rid]["l_cons"] = 0
	road_data[rid]["s_cons"] = 0
	road_data[rid]["last_l"] = 0
	road_data[rid]["last_s"] = 0

f_data = open(data_file, 'w')
json.dump(road_data, f_data, indent=1)
f_data.close()