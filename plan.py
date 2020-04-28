import gym
import json
import math


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
    def __init__(self, roadnet_config_file):
        with open(roadnet_config_file) as f:
            self.roadnet_json = json.load(f)

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

    def get_plan(self, start_road_id, end_road_id, speed, data):
        inters = self.intersection_list.copy()
        open_list = [self.road_list[start_road_id].end_inter_id]
        close_list = []
        while open_list != []:
            min_dist = 99999999
            for i in open_list:
                F = inters[i].G + inters[i].get_H(inters[self.road_list[end_road_id].start_inter_id], speed)
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
                    wait_time = (data[link[0]]["straight"] + data[link[0]]["left"])/2
                    inters[link[1]].G = inters[current].G + self.get_dist(inters[current], inters[link[1]])/speed + wait_time
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

    def generate_plan(self, origin_flow, gen_flow, road_data):
        with open(origin_flow) as f:
            self.flow_json = json.load(f)
        with open(road_data) as g:
            data = json.load(g)
        for vehicle in self.flow_json:
            start_id = vehicle["route"][0]
            end_id = vehicle["route"][-1]
            speed = vehicle["vehicle"]["maxSpeed"]
            plan = self.get_plan(start_id, end_id, speed, data)
            vehicle["route"] = plan
        f_gen = open(gen_flow, 'w')
        json.dump(self.flow_json, f_gen, indent=1)
        f_gen.close()


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


def main():
    a_star = A_Star_Plan("examples/hangzhou_4x4_gudang_18041610_1h/roadnet_4X4.json")
    a_star.generate_plan("examples/hangzhou_4x4_gudang_18041610_1h/hangzhou_4x4_gudang_18041610_1h.json", 
        "examples/hangzhou_4x4_gudang_18041610_1h/astar_record.json",
        "examples/hangzhou_4x4_gudang_18041610_1h/road_data.json")
    #clean_plan("examples/hangzhou_4x4_gudang_18041610_1h/hangzhou_4x4_gudang_18041610_1h.json", "examples/hangzhou_4x4_gudang_18041610_1h/noplan.json")

main()