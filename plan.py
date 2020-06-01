import gym
import json
import math
import os
from copy import deepcopy


class Intersection():
    def __init__(self, inter_obj):
        self.id = inter_obj.id
        self.pos = inter_obj.pos
        self.roadlinks = inter_obj.roadlinks
        self.father = None
        self.G = 0
        self.last_road = None


class Road():
    def __init__(self, road_id):
        self.id = road_id
        self.start_inter_id = None
        self.end_inter_id = None
        self.valid_outroads = []
        self.reverse = None

    def set_start_inter(self, inter_id):
        self.start_inter_id = inter_id

    def set_end_inter(self, inter_id):
        self.end_inter_id = inter_id

    def calculate(self, roads, inters):
        if self.end_inter_id:
            for link in inters[self.end_inter_id].roadlinks:
                if link[0] == self.id and link[1] not in self.valid_outroads:
                    self.valid_outroads.append(link[1])
                if roads[link[1]].end_inter_id == self.start_inter_id:
                    self.reverse = link[1]


class A_Star_Plan():
    def __init__(self, inters, flow_file):
        self.intersection_list = {}
        self.road_list = {}

        with open(flow_file) as f:
            self.flow_json = json.load(f)
        self.max_speed = self.flow_json[0]["vehicle"]["maxSpeed"]

        for inter_obj in inters:
            self.intersection_list[inter_obj.id] = Intersection(inter_obj)
            for link in inter_obj.roadlinks:
                if link[0] not in self.road_list:
                    self.road_list[link[0]] = Road(link[0])
                self.road_list[link[0]].set_end_inter(inter_obj.id)
                if link[1] not in self.road_list:
                    self.road_list[link[1]] = Road(link[1])
                self.road_list[link[1]].set_start_inter(inter_obj.id)

        for road in self.road_list:
            self.road_list[road].calculate(self.road_list, self.intersection_list)

    def get_dist(self, inter1, inter2):
        sq_dist = math.pow(inter1.pos["x"]-inter2.pos["x"], 2) + math.pow(inter1.pos["y"]-inter2.pos["y"], 2)
        dist = math.sqrt(sq_dist)
        return dist

    def delete_road(self, inters, roads, road_id):
        start_inter_id = self.road_list[road_id].start_inter_id
        end_inter_id = self.road_list[road_id].end_inter_id
        roadlinks_start = deepcopy(inters[start_inter_id].roadlinks)
        roadlinks_end = deepcopy(inters[end_inter_id].roadlinks)
        for link in inters[start_inter_id].roadlinks:
            if link[1] == road_id:
                roadlinks_start.remove(link)
        for link in inters[end_inter_id].roadlinks:
            if link[0] == road_id:
                roadlinks_end.remove(link)
        inters[start_inter_id].roadlinks = roadlinks_start
        inters[end_inter_id].roadlinks = roadlinks_end
        for road in roads:
            roads[road].calculate(roads, inters)

    def get_plan(self, start_road_id, end_road_id, record):
        if start_road_id == end_road_id:
            return [end_road_id]
        inters = deepcopy(self.intersection_list)
        roads = deepcopy(self.road_list)
        first_inter = roads[start_road_id].end_inter_id
        inters[first_inter].last_road = start_road_id
        open_list = [first_inter]
        close_list = []
        current = None
        while open_list != []:
            min_dist = 999999
            for i in open_list:
                F = inters[i].G + self.get_dist(inters[i], inters[roads[end_road_id].start_inter_id])/self.max_speed
                if F < min_dist:
                    current = i
                    min_dist = F

            if roads[inters[current].last_road].reverse == end_road_id:
                self.delete_road(inters, roads, inters[current].last_road)
                open_list.remove(current)
                continue
            if end_road_id in roads[inters[current].last_road].valid_outroads:
                break

            open_list.remove(current)
            close_list.append(current)
            if len(roads[inters[current].last_road].valid_outroads) != 3:
                print(len(roads[inters[current].last_road].valid_outroads))

            for outroad in roads[inters[current].last_road].valid_outroads:
                next_inter_id = roads[outroad].end_inter_id
                if next_inter_id == None:
                    continue
                if next_inter_id not in close_list and next_inter_id not in open_list:
                    open_list.append(next_inter_id)
                    inters[next_inter_id].father = inters[current]
                    inters[next_inter_id].last_road = outroad
                    if record.is_in_record(outroad):
                        inters[next_inter_id].G = inters[current].G + record.get_average_time(outroad)
                    else:
                        inters[next_inter_id].G = inters[current].G + self.get_dist(inters[current], inters[next_inter_id])/self.max_speed
                elif next_inter_id in open_list:
                    if inters[current].G + self.get_dist(inters[current], inters[next_inter_id])/self.max_speed < inters[next_inter_id].G:
                        inters[next_inter_id].father = inters[current]
                        inters[next_inter_id].last_road = outroad

        route_plan = [start_road_id]
        reverse_route = [end_road_id]
        i = roads[end_road_id].start_inter_id
        while i != roads[start_road_id].end_inter_id:
            reverse_route.append(inters[i].last_road)
            i = inters[i].father.id
        for ct in range(len(reverse_route)):
            route_plan.append(reverse_route[-1-ct])

        return route_plan


class Record():
    def __init__(self, interval, min_reference):
        self.interval = interval
        self.road_records = {}
        self.buffer = {}
        self.min_reference = min_reference

    def update(self, world, time):
        vehicles = world.eng.get_vehicles()
        for v in vehicles:
            info = world.eng.get_vehicle_info(v)
            if v not in self.buffer:
                if "road" in info:
                    self.buffer[v] = [time, info["road"], time]
            elif "road" in info and info["road"] == self.buffer[v][1]:
                self.buffer[v][2] = time
            else:
                if self.buffer[v][1] not in self.road_records:
                    self.road_records[self.buffer[v][1]] = [[time, self.buffer[v][2] - self.buffer[v][0]]]
                else:
                    self.road_records[self.buffer[v][1]].append([time, self.buffer[v][2] - self.buffer[v][0]])
                del self.buffer[v]

        for v in list(self.buffer.keys()):
            if v not in vehicles:
                del self.buffer[v]

        for road in self.road_records:
            while len(self.road_records[road]) > self.min_reference:
                if self.road_records[road][0][0] < time - self.interval:
                    del self.road_records[road][0]
                else:
                    break

    def is_in_record(self, road_id):
        return road_id in self.road_records

    def get_average_time(self, road_id):
        sum_time = 0
        sum_count = 0
        for entry in self.road_records[road_id]:
            sum_time += entry[1]
            sum_count += 1

        return 1.0*sum_time/sum_count

    def print_records(self):
        print(self.road_records)


class VehicleControl():
    def __init__(self):
        self.last_state = {}
        self.success = 0
        self.failure = 0

    def replan(self, world, plan, record):
        vehicles = world.eng.get_vehicles()
        for v in vehicles:
            info = world.eng.get_vehicle_info(v)
            if "road" in info:
                road = info["road"]
            else:
                road = '0'
            if v not in self.last_state:
                self.last_state[v] = road
                continue
            if self.last_state[v] == road or road == '0':
                continue
            
            route = info["route"].split(' ')[:-1]
            new_route = plan.get_plan(route[0], route[-1], record)
            if len(new_route) == 1:
                submit_route = new_route
            else:
                submit_route = new_route[1:]
            if new_route != route:
                if world.eng.set_vehicle_route(v, submit_route):
                    #print("---- old: ", route, "new: ", new_route)
                    self.success += 1
                else:
                    #print(route, new_route)
                    self.failure += 1
                    
    def summary(self):
        print("Replan Success: ", self.success, "Failure: ", self.failure)


def clean_plan(origin_flow, gen_flow):
        with open(origin_flow) as f:
            flow_json = json.load(f)
        for vehicle in flow_json:
            start_id = vehicle["route"][0]
            end_id = vehicle["route"][-1]
            vehicle["route"] = [start_id, end_id]
        f_gen = open(gen_flow, 'w')
        json.dump(flow_json, f_gen, indent=1)
        f_gen.close()