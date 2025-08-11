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

class FidelityEquation(JackalBase):
    def __init__(self, min_v=-1, max_v=2, min_w=-3.14, max_w=3.14, **kwargs):
        super().__init__(**kwargs)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.action_space = Box(
            low=np.full(20, 0.03),
            high=np.full(20, 0.15),
            dtype=np.float32
        )

    def _take_action(self, action):

        if np.sum(action) > 2.0:
            action = action * (2.0 / np.sum(action))

        action = np.clip(action, 0.03, 0.15)

        # Set the parameters
        self.gazebo_sim.unpause()
        self.jackal_ros.set_dynamics_equation(action)
        # Wait for robot to navigate for one time step
        rospy.sleep(self.time_step)
        self.gazebo_sim.pause()
