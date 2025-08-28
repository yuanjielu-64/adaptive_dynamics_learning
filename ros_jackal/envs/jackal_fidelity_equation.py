# jackal_laser.py
import gym
import numpy as np
from gym.spaces import Box

try:  # make sure to create a fake environment without ros installed
    import rospy
    from geometry_msgs.msg import Twist
except ModuleNotFoundError:
    pass

from envs import JackalBase

class FidelityEquation_continues(JackalBase):
    def __init__(self, min_v=-1, max_v=2, min_w=-3.14, max_w=3.14, **kwargs):
        super().__init__(**kwargs)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.param_init = self.param_list = np.array([
            0.0302, 0.0495, 0.0608, 0.0697, 0.0771,
            0.0835, 0.0893, 0.0946, 0.0994, 0.1039,
            0.1082, 0.1122, 0.116, 0.1196, 0.1231,
            0.1264, 0.1296, 0.1327, 0.1357, 0.1386
        ])

        self.action_space = Box(
            low=np.full(20, 0.02),
            high=np.full(20, 0.18),
            dtype=np.float32
        )

        self.last_action = None

    def _take_action(self, action):

        action = np.clip(action, 0.02, 0.18)

        if np.sum(action) > 2.0:
            action = action * (2.0 / np.sum(action))

        # Set the parameters
        self.gazebo_sim.unpause()
        self.jackal_ros.set_dynamics_equation(action)
        # Wait for robot to navigate for one time step
        rospy.sleep(self.time_step)
        self.gazebo_sim.pause()

        self.jackal_ros.last_action = action

RANGE_DICT = {
    'p': [1.0, 1.7],
    'total_time': [1.6, 2.2],
    'time_steps': [15, 22],
    'blend': [0.0, 0.6]
}

class FidelityEquation_function(JackalBase):
    def __init__(self, min_v=-1, max_v=2, min_w=-3.14, max_w=3.14, **kwargs):
        super().__init__(**kwargs)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.param_list = ['p', 'total_time', 'time_steps', 'blend']
        self.param_init = np.array([1.4, 2, 20, 0])

        self.action_space = Box(
            low=np.array([RANGE_DICT[k][0] for k in self.param_list]),
            high=np.array([RANGE_DICT[k][1] for k in self.param_list]),
            dtype=np.float32
        )

        self.last_action = self.param_init

    def _take_action(self, action):

        p, total_time, time_steps_float, blend = action

        time_steps = int(np.clip(round(time_steps_float),
                                 RANGE_DICT['time_steps'][0],
                                 RANGE_DICT['time_steps'][1]))

        time_intervals = []
        previous_time = 0.0
        for i in range(1, time_steps + 1):
            normalized_step = i / time_steps
            linear_time = normalized_step * total_time
            power_time = (normalized_step ** p) * total_time
            current_time = blend * linear_time + (1 - blend) * power_time

            interval = current_time - previous_time
            interval = round(interval * 10000.0) / 10000.0
            time_intervals.append(interval)
            previous_time = current_time

        # Set the parameters
        self.gazebo_sim.unpause()
        self.jackal_ros.set_dynamics_equation(time_intervals)
        # Wait for robot to navigate for one time step
        rospy.sleep(self.time_step)
        self.gazebo_sim.pause()

        self.jackal_ros.last_action = action
