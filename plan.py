import gym
import json
import math
import os


class Intersection():
    def __init__(self, intersection_json):
        self.id = intersection_json["id"]
        self.pos = intersection_json["point"]
        self.roads = intersection_json["roads"]
        self.outlinks = []
        self.inlinks = []
        self.virtual = intersection_json["virtual"]
        self.father = None
        self.G = 0

    def update_outlinks(self, road_id, inter_id):
        self.outlinks.append([road_id, inter_id])

    def update_inlinks(self, road_id, inter_id):
        self.inlinks.append([road_id, inter_id])

    def get_H(self, end_inter, speed):
        sq_dist = math.pow(self.pos["x"]-end_inter.pos["x"], 2) + math.pow(self.pos["y"]-end_inter.pos["y"], 2)
        dist = math.sqrt(sq_dist)/speed

        return dist


class Road():
    def __init__(self, road_id):
        self.id = road_id
        self.start_inter_id = None
        self.end_inter_id = None

    def set_start_inter(self, inter_id):
        self.start_inter_id = inter_id

    def set_end_inter(self, inter_id):
        self.end_inter_id = inter_id


class A_Star_Plan():
    def __init__(self, roadnet_file, flow_file):
        with open(roadnet_file) as f:
            self.roadnet_json = json.load(f)
        with open(flow_file) as g:
            self.flow_json = json.load(g)
        self.max_speed = self.flow_json[0]["vehicle"]["maxSpeed"]
        self.intersection_list = {}
        self.road_list = {}
        for inter in self.roadnet_json["intersections"]:
            self.intersection_list[inter["id"]] = Intersection(inter)
            for road_id in inter["roads"]:
                if road_id not in self.road_list:
                    self.road_list[road_id] = Road(road_id)
                if self.is_start_inter(inter["id"], road_id):
                    self.road_list[road_id].set_start_inter(inter["id"])
                else:
                    self.road_list[road_id].set_end_inter(inter["id"])

        for road_id in self.road_list:
            self.intersection_list[self.road_list[road_id].start_inter_id].update_outlinks(road_id, self.road_list[road_id].end_inter_id)
            self.intersection_list[self.road_list[road_id].end_inter_id].update_inlinks(road_id, self.road_list[road_id].start_inter_id)

    def is_start_inter(self, inter_id, road_id):
        road_id_list = road_id.split('_')
        inter_id_list = inter_id.split('_')
        if road_id_list[1] == inter_id_list[1] and road_id_list[2] == inter_id_list[2]:
            return True
        else:
            return False

    def get_dist(self, inter1, inter2):
        sq_dist = math.pow(inter1.pos["x"]-inter2.pos["x"], 2) + math.pow(inter1.pos["y"]-inter2.pos["y"], 2)
        dist = math.sqrt(sq_dist)
        return dist

    def get_father_route(self, inter):
        father = inter.father
        for link in father.outlinks:
            if link[1] == inter.id:
                return link[0]
        return "Not found"

    def get_plan(self, start_road_id, end_road_id, record):
        if start_road_id == end_road_id:
            return [end_road_id]
        inters = self.intersection_list.copy()
        open_list = [self.road_list[start_road_id].end_inter_id]
        close_list = []
        while open_list != []:
            min_dist = 99999999
            for i in open_list:
                F = inters[i].G + inters[i].get_H(inters[self.road_list[end_road_id].start_inter_id], self.max_speed)
                if F < min_dist:
                    current = i
                    min_dist = F
            if current == self.road_list[end_road_id].start_inter_id:
                break
            open_list.remove(current)
            close_list.append(current)
            for link in inters[current].outlinks:
                if inters[link[1]].virtual == False and link[1] not in close_list and link[1] not in open_list:
                    open_list.append(link[1])
                    inters[link[1]].father = inters[current]
                    if record.is_in_record(link[0]):
                        inters[link[1]].G = inters[current].G + record.get_average_time(link[0])
                    else:
                        inters[link[1]].G = inters[current].G + self.get_dist(inters[current], inters[link[1]])/self.max_speed
                elif link[1] in open_list:
                    if inters[current].G + self.get_dist(inters[current], inters[link[1]]) < inters[link[1]].G:
                        inters[link[1]].father = inters[current]

        route_plan = [start_road_id]
        reverse_route = [end_road_id]
        i = self.road_list[end_road_id].start_inter_id
        while i != self.road_list[start_road_id].end_inter_id:
            reverse_route.append(self.get_father_route(inters[i]))
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
        if not self.is_in_record(road_id):
            print("Road has no record")
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
            if new_route != route:
                #print("New plan")
                if not world.eng.set_vehicle_route(v, new_route[1:]):
                    self.failure += 1
                    #print(route)
                    #print(new_route)
                else:
                    self.success += 1

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