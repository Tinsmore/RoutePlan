import random
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam, RMSprop, SGD
import os


class IntersectionAgent():
    """
    Agent selecting the best direction in real time
    """
    def __init__(self, inter):
        self.id = inter.id
        self.outroads = inter.out_roads
        self.ob_length = len(inter.out_roads)

        self.memory = deque(maxlen=200)
        self.learning_start = 100
        self.remember_freq = 30
        self.update_model_freq = 300
        self.update_target_model_freq = 300
        
        self.gamma = 0.95  # discount rate
        self.learning_rate = 0.005
        self.batch_size = 8

        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_network()

        self.lanes = self.get_lanes()
        self.index = self.get_index()
        self.ob = []
        self.next_ob = []
        self.action = None
        self.reward = None

    def build_model(self):
        model = Sequential()
        model.add(Dense(8, input_dim=self.ob_length, activation='relu'))
        model.add(Dense(self.ob_length, activation='relu'))
        model.compile(
            loss='mse',
            optimizer=RMSprop()
        )
        #model.summary()
        return model

    def _reshape_ob(self, ob):
        return np.reshape(ob, (1, -1))

    def get_lanes(self):
        lanes = {}
        for road in self.outroads:
            num = 0
            lanes_of_road = []
            for lane in road['lanes']:
                lane_id = road['id'] + '_' + str(num)
                lanes_of_road.append(lane_id)
                num += 1
            lanes[road['id']] = lanes_of_road
        return lanes

    def get_index(self):
        index = []
        for road in self.outroads:
            index.append(road['id'])
        return index

    def get_ob(self, waiting):
        ob = []
        for road in self.lanes:
            road_total = 0
            for lane_id in self.lanes[road]:
                road_total += waiting[lane_id]
            ob.append(road_total)
        return ob

    def get_action(self, waiting):
        ob = self.get_ob(waiting)
        ob = self._reshape_ob(ob)
        act_values = self.model.predict(ob)
        return np.argmax(act_values[0])

    def get_reward(self, waiting):
        reward = 0
        for road in self.lanes:
            road_total = 0
            for lane_id in self.lanes[road]:
                reward += waiting[lane_id]
        return reward

    def update_target_network(self):
        weights = self.model.get_weights()
        self.target_model.set_weights(weights)

    def remember(self, waiting, time):
        if time % self.remember_freq == self.remember_freq - 1:
            next_ob = self.get_ob(waiting)
            reward = self.get_reward(waiting)
            if len(self.ob)>0 and self.action != None:
                self.memory.append((self.ob, self.action, reward, next_ob))
            self.ob = next_ob
            self.action = self.get_action(waiting)

    def replay(self):
        minibatch = random.sample(self.memory, self.batch_size)
        obs, actions, rewards, next_obs = [np.stack(x) for x in np.array(minibatch).T]
        target = rewards + self.gamma * np.amax(self.target_model.predict(next_obs), axis=1)
        target_f = self.model.predict(obs)
        for i, action in enumerate(actions):
            target_f[i][action] = target[i]
        history = self.model.fit(obs, target_f, epochs=1, verbose=0)

    def load_model(self, dir="model"):
        if not os.path.exists(dir):
            return
        name = "inter_agent_{}.h5".format(self.id)
        model_name = os.path.join(dir, name)
        self.model.load_weights(model_name)

    def save_model(self, dir="model"):
        if not os.path.exists(dir):
            os.makedirs(dir)
        name = "inter_agent_{}.h5".format(self.id)
        model_name = os.path.join(dir, name)
        self.model.save_weights(model_name)