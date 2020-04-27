import gym
import json


class Intersection():
    def __init__(self, intersection_json):
        self.id = intersection_json["id"]
        self.pos = intersection_json["point"]
        self.roads = intersection_json["roads"]
        self.outlinks = []
        self.inlinks = []

    def update_outlinks(self, road_id, inter_id):
        self.outlinks.append([road_id, inter_id])

    def update_inlinks(self, road_id, inter_id):
        self.inlinks.append([road_id, inter_id])


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

    def get_plan(self, start_road_id, end_road_id):
        route_plan = [start_road_id]
        current_road_id = start_road_id
        ct = 0
        while current_road_id != end_road_id and ct < 4:
            ct += 1
            inter_ahead = self.road_list[current_road_id].end_inter_id
            available_links = self.intersection_list[inter_ahead].outlinks
            if available_links == []:
                break
            chosen_link = available_links[0]
            current_road_id = chosen_link[0]
            route_plan.append(current_road_id)
        route_plan.append(end_road_id)

        return route_plan

    def generate_plan(self, origin_flow, gen_flow):
        with open(origin_flow) as f:
            self.flow_json = json.load(f)
        for vehicle in self.flow_json:
            start_id = vehicle["route"][0]
            end_id = vehicle["route"][-1]
            vehicle["route"] = self.get_plan(start_id, end_id)
        f_gen = open(gen_flow, 'w')
        json.dump(self.flow_json, f_gen, indent=1)
        f_gen.close()


def main():
    a_star = A_Star_Plan("examples/hangzhou_4x4_gudang_18041610_1h/roadnet_4X4.json")
    a_star.generate_plan("examples/hangzhou_4x4_gudang_18041610_1h/hangzhou_4x4_gudang_18041610_1h.json", "examples/hangzhou_4x4_gudang_18041610_1h/astar_1h.json")


main()