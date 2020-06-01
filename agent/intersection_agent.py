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

        self.memory = deque(maxlen=2000)
        self.learning_start = 100
        self.update_model_freq = 10
        self.update_target_model_freq = 20

        self.gamma = 0.95  # discount rate
        self.epsilon = 0.1  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.99
        self.learning_rate = 0.005
        self.batch_size = 32

        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_network()

    def build_model(self):
        model = Sequential()
        model.add(Dense(8, input_dim=self.ob_length, activation='relu'))
        model.add(Dense(self.ob_length, activation='linear'))
        model.compile(
            loss='mse',
            optimizer=RMSprop()
        )
        #model.summary()
        return model

    def _reshape_ob(self, ob):
        return np.reshape(ob, (1, -1))

    def get_ob(self, running):
        ob = []
        for road in self.outroads:
            num = 0
            road_total = 0
            prefix = road['id']
            for lane in road['lanes']:
                lane_num = prefix + '_' + str(num)
                road_total += running[lane_num]
                num += 1
            ob.append(running[lane_num])
        return ob

    def get_values(self, running, time):
        if np.random.rand() <= self.epsilon:
            return [1]*self.ob_length
        ob = self.get_ob(running)
        ob = self._reshape_ob(ob)
        act_values = self.model.predict(ob)
        return act_values

    def update_target_network(self):
        weights = self.model.get_weights()
        self.target_model.set_weights(weights)

    def remember(self, ob, action, reward, next_ob):
        self.memory.append((ob, action, reward, next_ob))

    def replay(self):
        minibatch = random.sample(self.memory, self.batch_size)
        obs, actions, rewards, next_obs = [np.stack(x) for x in np.array(minibatch).T]
        target = rewards + self.gamma * np.amax(self.target_model.predict(next_obs), axis=1)
        target_f = self.model.predict(obs)
        for i, action in enumerate(actions):
            target_f[i][action] = target[i]
        history = self.model.fit(obs, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load_model(self, dir="model"):
        name = "inter_agent_{}.h5".format(self.id)
        model_name = os.path.join(dir, name)
        self.model.load_weights(model_name)

    def save_model(self, dir="model"):
        name = "inter_agent_{}.h5".format(self.id)
        model_name = os.path.join(dir, name)
        self.model.save_weights(model_name)