import gym
from environment import TSCEnv
from world import World
from generator import LaneVehicleGenerator
from agent import MaxPressureAgent, IntersectionAgent
from metric import TravelTimeMetric
import argparse
from plan import *
import os.path as osp

# parse args
parser = argparse.ArgumentParser(description='Run Example')
parser.add_argument('config_file', type=str, help='path of config file')
parser.add_argument('--thread', type=int, default=8, help='number of threads')
parser.add_argument('--episodes', type=int, default=10, help='number of episodes')
parser.add_argument('--steps', type=int, default=3600, help='number of steps')
args = parser.parse_args()

# open files
with open(args.config_file) as f:
    config = json.load(f)
roadnet_file = osp.join(config["dir"], config["roadnetFile"])
flow_file = osp.join(config["dir"], config["flowFile"])

# create world
world = World(args.config_file, thread_num=args.thread)

# create agents
agents = []
for i in world.intersections:
    action_space = gym.spaces.Discrete(len(i.phases))
    agents.append(MaxPressureAgent(
        action_space, i, world, 
        LaneVehicleGenerator(world, i, ["lane_count"], in_only=True)
    ))

# route plan agents
guide_agents = []
with open(roadnet_file) as rf:
    roadnet = json.load(rf)
for inter in world.intersections:
    guide_agents.append(IntersectionAgent(inter))

# create metric
metric = TravelTimeMetric(world)

# create env
env = TSCEnv(world, agents, metric)

# Plan agent
a_star_plan = A_Star_Plan(world.intersections, flow_file)

# Record agent
record = Record(interval=300, min_reference=5)

# Vehicle control agent
vc = VehicleControl()

def train():
    for e in range(args.episodes):
        obs = env.reset()
        for i in range(args.steps):
            record.update(world, i)
            vc.replan(world, a_star_plan, record)
            actions = []
            for agent_id, agent in enumerate(agents):
                actions.append(agent.get_action(obs[agent_id]))
            obs, rewards, dones, info = env.step(actions)

            running_v = world.eng.get_lane_vehicle_count()
            for agent in guide_agents:
                Q_values = agent.get_values(running_v, i)

            total_time = e*args.steps + i
            for agent in guide_agents:
                if total_time > agent.learning_start and total_time % agent.update_model_freq == agent.update_model_freq - 1:
                    agent.replay()
                if total_time > agent.learning_start and total_time % agent.update_target_model_freq == agent.update_target_model_freq - 1:
                    agent.update_target_network()

        vc.summary()
        print("Episode: ", e, " Final travel time: ", env.eng.get_average_travel_time())

def test():
    obs = env.reset()
    for i in range(args.steps):
        record.update(world, i)
        vc.replan(world, a_star_plan, record)
        actions = []
        for agent_id, agent in enumerate(agents):
            actions.append(agent.get_action(obs[agent_id]))
        obs, rewards, dones, info = env.step(actions)

    vc.summary()
    print("Final travel time: ", env.eng.get_average_travel_time())

def main():
    train()

main()