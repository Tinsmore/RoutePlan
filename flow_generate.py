import json
import math
import os
import numpy as np


flow_file = "examples/manhattan_16x3/anon_16_3_newyork_real.json"
# "examples/hangzhou_4x4_gudang_18041610_1h/hangzhou_4x4_gudang_18041610_1h.json"
# "examples/manhattan_16x3/anon_16_3_newyork_real.json"
# "examples/manhattan_28x7/anon_28_7_newyork_real_double.json"

#Dense
dense_flow_file = "examples/manhattan_16x3/anon_16_3_newyork_real_d.json"
# "examples/hangzhou_4x4_gudang_18041610_1h/hangzhou_4x4_gudang_18041610_1h_d.json"
# "examples/manhattan_16x3/anon_16_3_newyork_real_d.json"
# "examples/manhattan_28x7/anon_28_7_newyork_real_double_d.json"

with open(flow_file) as f:
	flow = json.load(f)

for vehicle in flow:
    vehicle["endTime"] = vehicle["endTime"] + 2

f_gen = open(dense_flow_file, 'w')
json.dump(flow, f_gen, indent=1)
f_gen.close()