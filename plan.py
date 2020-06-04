import gym
import json
import math
import os
import numpy as np
from copy import deepcopy


class Intersection():
    def __init__(self, inter_json):
        self.id = inter_json["id"]
        self.pos = inter_json["point"]

        links = inter_json["roadLinks"]
        self.roadlinks = []
        for link in links:
            self.roadlinks.append([link["startRoad"], link["endRoad"]])


class Road():
    def __init__(self, road_json):
        self.id = road_json["id"]
        points = road_json["points"]
        sq_dist = math.pow(points[0]["x"] - points[1]["x"], 2) + math.pow(points[0]["y"] - points[1]["y"], 2)
        self.length = math.sqrt(sq_dist)

        self.start_inter_id = road_json["startIntersection"]
        self.end_inter_id = road_json["endIntersection"]

        speed_sum = 0
        speed_count = 0
        for lane in road_json["lanes"]:
            speed_sum += lane["maxSpeed"]
            speed_count += 1
        self.max_speed = 1.0*speed_sum/speed_count
        self.shortest_time = 1.0*self.length/self.max_speed

        self.valid_outroads = []
        self.father = None
        self.G = 0

    def calculate(self, roads, inters):
        if self.end_inter_id:
            for link in inters[self.end_inter_id].roadlinks:
                if link[0] == self.id:
                    self.valid_outroads.append(link[1])


class A_Star_Plan():
    def __init__(self, roadnet_file):
        self.intersection_list = {}
        self.road_list = {}

        with open(roadnet_file) as f:
            roadnet = json.load(f)

        for inter in roadnet["intersections"]:
            self.intersection_list[inter["id"]] = Intersection(inter)
        for road in roadnet["roads"]:
            self.road_list[road["id"]] = Road(road)
        for road in self.road_list:
            self.road_list[road].calculate(self.road_list, self.intersection_list)

    def get_H(self, road1, road2):
        inter1 = self.intersection_list[road1.end_inter_id]
        inter2 = self.intersection_list[road2.start_inter_id]
        sq_dist = math.pow(inter1.pos["x"]-inter2.pos["x"], 2) + math.pow(inter1.pos["y"]-inter2.pos["y"], 2)
        dist = math.sqrt(sq_dist)
        return dist

    def get_plan(self, start_road_id, end_road_id, record):
        if start_road_id == end_road_id:
            return 0, [end_road_id]

        roads = deepcopy(self.road_list)
        open_list = [start_road_id]
        close_list = []
        current = None
        while open_list != []:
            min_dist = 999999
            for i in open_list:
                F = roads[i].G + self.get_H(roads[i], roads[end_road_id])/roads[i].max_speed
                if F < min_dist:
                    current = i
                    min_dist = F
            if current == end_road_id:
                break
            open_list.remove(current)
            close_list.append(current)
            for outroad in roads[current].valid_outroads:
                if outroad not in close_list and outroad not in open_list:
                    open_list.append(outroad)
                    roads[outroad].father = current
                    roads[outroad].G = roads[current].G + record.get_average_time(outroad)
                elif outroad in open_list:
                    if roads[current].G + roads[outroad].shortest_time < roads[outroad].G:
                        roads[outroad].father = current
                        roads[outroad].G = roads[current].G + roads[outroad].shortest_time

        route_plan = [start_road_id]
        reverse_route = [end_road_id]
        r = roads[end_road_id].father
        while r != start_road_id:
            reverse_route.append(r)
            r = roads[r].father
        for ct in range(len(reverse_route)):
            route_plan.append(reverse_route[-1-ct])
        last_G = roads[end_road_id].G

        return last_G, route_plan


class Record():
    def __init__(self, plan, interval, min_reference):
        self.interval = interval
        self.road_records = {}
        self.buffer = {}
        self.min_reference = min_reference
        self.road_list = plan.road_list

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

    def get_average_time(self, road_id):
        if road_id not in self.road_records:
            return self.road_list[road_id].shortest_time
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

    def replan(self, world, plan, record, suggestions, indexs):
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
            self.last_state[v] = road
            next_inter = info["intersection"]
            route = info["route"].split(' ')[:-1]
            if len(route) <= 1:
                continue

            new_tta, new_route = plan.get_plan(route[0], route[-1], record)
            max_road = indexs[next_inter][suggestions[next_inter]]
            if new_route[1] != max_road or new_route == route:
                continue
            if world.eng.set_vehicle_route(v, new_route[1:]):
                self.success += 1
            else:
                self.failure += 1
                    
    def reset(self):
        self.last_state = {}
        self.success = 0
        self.failure = 0
        
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