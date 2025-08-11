from gym.spaces import Box
import numpy as np

try:  # make sure to create a fake environment without ros installed
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray
except ModuleNotFoundError:
    pass

from envs.jackal_gazebo_envs import JackalGazebo, JackalGazeboLaser

class MotionControlContinuous(JackalGazebo):
    def __init__(self, min_v=-1, max_v=2, min_w=-3.14, max_w=3.14, **kwargs):
        self.action_dim = 20
        super().__init__(**kwargs)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._action_array_pub = rospy.Publisher('/array_dt', Float64MultiArray, queue_size=1)

        self.range_dict = {
            "dt": [0.0, 0.1]  # 时间步长范围为0到0.1秒
        }

        self.action_space = Box(
            low=np.zeros(self.action_dim),  # 所有维度的下限都是0.0
            high=np.ones(self.action_dim) * 0.1,  # 所有维度的上限都是0.1
            dtype=np.float32
        )

    # def reset(self):
    #     """reset the environment without setting the goal
    #     set_goal is replaced with make_plan
    #     """
    #     self.step_count = 0
    #     # Reset robot in odom frame clear_costmap
    #     self.gazebo_sim.unpause()
    #     # Resets the state of the environment and returns an initial observation
    #     self.gazebo_sim.reset()
    #     self._reset_move_base()
    #     self.start_time = self.current_time = rospy.get_time()
    #     obs = self._get_observation()
    #     self.gazebo_sim.pause()
    #     self.collision_count = 0
    #
    #     # self.collision_count = 0
    #     # # Reset robot in odom frame clear_costmap
    #     # self.gazebo_sim.reset()
    #     # self.start_time = self.current_time = rospy.get_time()
    #     # pos, psi = self._get_pos_psi()
    #     #
    #     # self.gazebo_sim.unpause()
    #     # obs = self._get_observation(pos, psi, np.array([0, 0]))
    #     # self.gazebo_sim.pause()
    #     #
    #     # goal_pos = np.array([self.world_frame_goal[0] - pos.x, self.world_frame_goal[1] - pos.y])
    #     # self.last_goal_pos = goal_pos
    #     return obs

    def _take_action(self, action):
        # linear_speed, angular_speed = action
        # cmd_vel_value = Twist()
        # cmd_vel_value.linear.x = linear_speed
        # cmd_vel_value.angular.z = angular_speed

        action_msg = Float64MultiArray()
        action_msg.data = action.astype(np.float64).tolist()  # Convert to float64
        self._action_array_pub.publish(action_msg)

        self.gazebo_sim.unpause()
        # self._cmd_vel_pub.publish(cmd_vel_value)
        self.gazebo_sim.pause()

class MotionControlContinuousLaser(MotionControlContinuous, JackalGazeboLaser):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
