import os

from gitdb.util import mkdir
import yaml
import pickle
from os.path import join, dirname, abspath, exists
import sys

sys.path.append(dirname(dirname(abspath(__file__))))
import torch
import gym
import numpy as np
import random
import time
import rospy
import argparse
import logging

from td3.train import initialize_policy
from envs import registration
from envs.wrappers import ShapingRewardWrapper, StackFrame

os.environ["JACKAL_LASER"] = "1"
os.environ["JACKAL_LASER_MODEL"] = "ust10"
os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"

class FileSync:
    def __init__(self, actor_id, buffer_path, actor_dir):
        self.actor_id = actor_id
        self.sync_dir = join(buffer_path, 'sync')
        os.makedirs(self.sync_dir, exist_ok=True)

        self.test_sync_dir = join(buffer_path, 'test_sync')
        os.makedirs(self.test_sync_dir, exist_ok=True)

        self.continue_file = join(self.sync_dir, 'continue.signal')
        self.actor_file = join(self.sync_dir, f'actor_{actor_id}.done')

        self.actor_dir = actor_dir

        self.last_file_time = 0
        self.train_limit = 4
        self.test_limit = 1

        self.status = 'stop'
        self.train_episode = 0
        self.test_episode = 0

    def wait_for_continue(self, opt_time, nav_metric, traj, id, path):

        self._read_command()

        if self.status == 'train':
            self.test_episode = 0

            self.train_episode += 1

            if self.train_episode == self.train_limit:
                # self._write_actor_status()
                self.write_buffer(opt_time, nav_metric, traj, self.train_episode, id, path, 'train')
                return False

            elif self.train_episode > self.train_limit:
                while True:
                    self._read_command()

                    if self.status == 'test' or self.status == 'pause':
                        return True

                    time.sleep(0.5)
            else:
                self.write_buffer(opt_time, nav_metric, traj, self.train_episode, id, path, 'train')
                return True

        elif self.status == 'pause':

            self.test_episode = 0
            self.train_episode = 0

            while True:
                self._read_command()

                if self.status == 'train' or self.status == 'test':
                    self.train_episode = 0
                    self.test_episode = 0
                    return True

                time.sleep(0.5)

        else:

            self.train_episode = 0

            self.test_episode += 1

            if self.test_episode == self.test_limit:
                self._write_actor_status()
                self.write_buffer(opt_time, nav_metric, traj, self.test_episode, id, self.test_sync_dir, 'test')
                return False

            elif self.test_episode > self.test_limit:
                while True:
                    self._read_command()

                    if self.status == 'train' or self.status == 'pause':
                        return True

                    time.sleep(0.5)
            else:
                self.write_buffer(opt_time, nav_metric, traj, self.test_episode, id, self.test_sync_dir, 'test')
                return True

    def _read_command(self):

        if not os.path.exists(self.continue_file):
            raise FileNotFoundError

        with open(self.continue_file, 'r') as f:
            command = f.readline().strip()

        self.status = command

    def _write_actor_status(self):
        if self.status == 'train':
            status = f"{self.status}:{self.train_episode}"
            with open(self.actor_file, 'w') as f:
                f.write(status)

        elif self.status == 'test':
            status = f"{self.status}:{self.test_episode}"
            with open(self.actor_file, 'w') as f:
                f.write(status)

    def write_buffer(self, opt_time, nav_metric, traj, ep, id, path, type):
        # actor_dir = join(BUFFER_PATH, 'actor_%s' % (str(id)))
        # # pickle_dir = join(actor_dir, 'pickle_file')
        # os.makedirs(actor_dir, exist_ok=True)

        if not traj:
            return

        if len(traj[-1]) < 5:
            return

        if len(traj) <= 1:
            return

        if type == 'train':
            total_reward = 0
            for i in range(len(traj)):
                rew = traj[i][2]
                total_reward += rew

            info_dict = traj[-1][4]

            if (info_dict['recovery'] == 1.0 and info_dict['status'] == 'timeout') or (info_dict['time'] >= 70):
                error_dir = os.path.join(BUFFER_PATH, 'actor_error')
                os.makedirs(error_dir, exist_ok=True)

                error_file = os.path.join(error_dir, f'{id}.txt')

                with open(error_file, 'a') as f:
                    f.write(
                        f"Environment {id} and World_name {info_dict['world']} has KeyError in info_dict, time: {info_dict['time']}, recovery: {info_dict['recovery']}, status: {info_dict['status']}\n")

                return

            with open(join(path, "trajectory_results.txt"), 'a') as f:

                f.write(
                    f"Train: Collision: {info_dict['collision']}, Recovery: {info_dict['recovery']:.6f}, Smoothness: {info_dict['smoothness']:.6f}, Status: {info_dict['status']}, Time: {info_dict['time']:.3f} , Reward: {total_reward:.3f}, Opt_time: {opt_time:.3f} , Nav_Metric: {nav_metric:.3f} , World: {info_dict['world']}\n")

            with open(join(path, 'traj_%d.pickle' % (ep)), 'wb') as f:
                try:
                    pickle.dump(traj, f)
                except OSError as e:
                    logging.exception('Failed to dump the trajectory! %s', e)
                    pass
        else:

            info_dict = traj[-1][4]

            if (info_dict['recovery'] == 1.0 and info_dict['status'] == 'timeout') or (info_dict['time'] >= 70):
                error_dir = os.path.join(BUFFER_PATH, 'actor_error')
                os.makedirs(error_dir, exist_ok=True)

                error_file = os.path.join(error_dir, f'{id}.txt')

                with open(error_file, 'a') as f:
                    f.write(
                        f"Test environment {id} has KeyError in info_dict, time: {info_dict['time']}, recovery: {info_dict['recovery']}, status: {info_dict['status']}\n")

                return

            total_reward = 0
            for i in range(len(traj)):
                rew = traj[i][2]
                total_reward += rew

            with open(join(self.actor_dir, "trajectory_results.txt"), 'a') as f:

                f.write(
                    f"Test: Collision: {info_dict['collision']}, Recovery: {info_dict['recovery']:.6f}, Smoothness: {info_dict['smoothness']:.6f}, Status: {info_dict['status']}, Time: {info_dict['time']:.3f} , Reward: {total_reward:.3f}, Opt_time: {opt_time:.3f} , Nav_Metric: {nav_metric:.3f} , World: {info_dict['world']}\n")

            with open(join(path, 'test_%d_%d.pickle' % (id, ep)), 'wb') as f:
                try:
                    pickle.dump(traj, f)
                except OSError as e:
                    logging.exception('Failed to dump the trajectory! %s', e)
                    pass

def initialize_actor(id, BUFFER_PATH):
    rospy.logwarn(">>>>>>>>>>>>>>>>>> actor id: %s <<<<<<<<<<<<<<<<<<" %(str(id)))
    assert os.path.exists(BUFFER_PATH), BUFFER_PATH
    actor_path = join(BUFFER_PATH, 'actor_%s' %(str(id)))

    if not exists(actor_path):
        os.mkdir(actor_path) # path to store all the trajectories

    f = None
    while f is None:
        try:
            f = open(join(BUFFER_PATH, 'config.yaml'), 'r')
        except:
            rospy.logwarn("wait for critor to be initialized")
            time.sleep(2)

    config = yaml.load(f, Loader=yaml.FullLoader)

    return config

def load_policy(policy):
    f = True
    while f:
        try:
            if not os.path.exists(join(BUFFER_PATH, "policy_copy_actor")):
                policy.load(BUFFER_PATH, "policy")
            f = False
        except FileNotFoundError:
            time.sleep(1)
        except:
            logging.exception('')
            time.sleep(1)

    return policy

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)

def get_score(INIT_POSITION, GOAL_POSITION, status, time, world):
    success = False

    if status == "success":
        success = True
    else:
        success = False

    world = int(world.split('_')[1].split('.')[0])

    path_file_name = join(WORLD_PATH, "path_files/", "path_%d.npy" % int(world))
    path_array = np.load(path_file_name)
    path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
    path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
    path_array = np.insert(path_array, len(path_array),
                           (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
    path_length = 0
    for p1, p2 in zip(path_array[:-1], path_array[1:]):
        path_length += compute_distance(p1, p2)

    optimal_time = path_length / 2
    actual_time = time
    nav_metric = int(success) * optimal_time / np.clip(actual_time, 2 * optimal_time, 8 * optimal_time)

    return optimal_time, nav_metric

def get_world_name(config, id):
    world_name = config["condor_config"]["worlds"][id]
    if isinstance(world_name, int):
        world_name = "world_%d.world" %(world_name)
    return world_name

def _debug_print_robot_status(env, count, rew, actions):
    Y = env.jackal_ros.get_robot_state()[1]
    X = env.jackal_ros.get_robot_state()[0]
    p = env.gazebo_sim.get_model_state().pose.position
    print(actions)
    print('current step: %d, X position: %f(world_frame), %f(odem_frame), Y position: %f(world_frame), %f(odom_frame), rew: %f' %(count, p.x, X, p.y, Y , rew))

def _update_reward(traj):
    failure_reward = traj[-1][2]
    failure_steps = min(4, len(traj))

    for i in range(failure_steps):
        step_idx = len(traj) - 1 - i

        penalty_ratio = 0.5 ** i
        adjusted_reward = failure_reward * penalty_ratio

        traj[step_idx][2] = adjusted_reward

    return traj

def main(id):

    actor_dir = join(BUFFER_PATH, 'actor_%s' % (str(id)))
    os.makedirs(actor_dir, exist_ok=True)

    file_sync = FileSync(id, BUFFER_PATH, actor_dir)

    config = initialize_actor(id, BUFFER_PATH)
    env_config = config['env_config']
    world_name = get_world_name(config, id)
    env_config["kwargs"]["world_name"] = world_name

    init_pos = env_config["kwargs"]["init_position"]
    goal_pos = env_config["kwargs"]["goal_position"]

    env = gym.make(env_config["env_id"], **env_config["kwargs"])
    if env_config["shaping_reward"]:
        env = ShapingRewardWrapper(env)
    env = StackFrame(env, stack_frame=env_config["stack_frame"])

    policy, _ = initialize_policy(config, env)

    flag = True

    print(">>>>>>>>>>>>>> Running on %s <<<<<<<<<<<<<<<<" %(world_name))

    while True:
        obs = env.reset()

        traj = []
        done = False
        policy = load_policy(policy)

        while not done and flag == True:
            act = policy.select_action(obs)
            obs_new, rew, done, info = env.step(act)
            info["world"] = world_name
            traj.append([obs, act, rew, done, info, 0, 0])
            obs = obs_new

        if flag == True:
            info_dict = traj[-1][4]
            opt_time, nav_metric = get_score(init_pos, goal_pos, info_dict['status'], info_dict['time'], info_dict['world'])

            traj[-1][5] = opt_time
            traj[-1][6] = nav_metric

            if traj[-1][4]['status'] == 'flip' or traj[-1][4]['status'] == 'timeout' or traj[-1][4]['collision'] >= 1:
                traj = _update_reward(traj)
        else:
            opt_time = nav_metric = 0

        flag = file_sync.wait_for_continue(opt_time, nav_metric, traj, id, actor_dir)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'start an actor')
    parser.add_argument('--id', dest='actor_id', type = int, default = 0)
    parser.add_argument('--policy_name', dest='policy_name', default="ddp_function")
    parser.add_argument('--buffer_path', dest='buffer_path', default="../buffer/")
    parser.add_argument('--world_path', dest='world_path', default="../jackal_helper/worlds/BARN/")

    args = parser.parse_args()
    BUFFER_PATH = args.buffer_path
    WORLD_PATH = args.world_path

    if (os.path.exists(BUFFER_PATH + args.policy_name) == False):
        os.makedirs(BUFFER_PATH + args.policy_name, exist_ok=True)

    BUFFER_PATH = BUFFER_PATH + args.policy_name
    id = args.actor_id
    main(id)
