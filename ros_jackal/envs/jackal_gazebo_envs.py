import gym
import time
import numpy as np
import os
from os.path import join
import subprocess
from gym.spaces import Box

try:  # make sure to create a fake environment without ros installed
    import rospy
    import rospkg
except ModuleNotFoundError:
    pass

from envs.utils.gazebo_simulation import GazeboSimulation
from envs.utils.move_base import MoveBase

class JackalGazebo(gym.Env):
    def __init__(
        self,
        world_name="jackal_world.world",
        gui=False,
        init_position=[-2, 3, 0],
        goal_position=[4, 0, 0],
        max_step=100,
        time_step=1,
        slack_reward=-1,
        failure_reward=-50,
        success_reward=0,
        collision_reward=0,
        goal_reward=1,
        max_collision=10000,
        verbose=True,
    ):
        """Base RL env that initialize jackal simulation in Gazebo
        """
        super().__init__()
        # config
        self.gui = gui
        self.verbose = verbose

        # sim config
        self.world_name = world_name

        # env config
        self.time_step = time_step
        self.max_step = max_step

        # reward
        self.slack_reward = slack_reward
        self.failure_reward = failure_reward
        self.success_reward = success_reward
        self.collision_reward = collision_reward
        self.goal_reward = goal_reward
        self.max_collision = max_collision

        self.launch_gazebo(world_name=self.world_name, gui=self.gui, verbose=self.verbose)
        self.set_start_goal_BARN(init_position, goal_position)
        self.launch_move_base(goal_position=self.goal_position,
                                                   base_local_planner="base_local_planner/TrajectoryPlannerROS")

        # place holders
        self.action_space = None
        self.observation_space = None

        self.step_count = 0
        self.collision_count = 0
        self.collided = 0
        self.start_time = self.current_time = None

    def launch_move_base(self, goal_position, base_local_planner):
        rospack = rospkg.RosPack()
        self.BASE_PATH = rospack.get_path('jackal_helper')
        launch_file = join(self.BASE_PATH, 'launch', 'move_base_launch.launch')
        self.move_base_process = subprocess.Popen(['roslaunch', launch_file, 'base_local_planner:=' + base_local_planner])
        self.move_base = MoveBase(goal_position=goal_position, base_local_planner=base_local_planner)

    def launch_gazebo(self, world_name, gui, verbose):
        # launch gazebo
        rospy.logwarn(">>>>>>>>>>>>>>>>>> Load world: %s <<<<<<<<<<<<<<<<<<" %(world_name))

        ros_package_path = os.environ.get('ROS_PACKAGE_PATH', 'Not set')
        print(f"ROS_PACKAGE_PATH: {ros_package_path}")

        rospack = rospkg.RosPack()
        self.BASE_PATH = rospack.get_path('jackal_helper')
        world_name = join(self.BASE_PATH, "worlds/BARN/", world_name)
        launch_file = join(self.BASE_PATH, 'launch', 'gazebo_launch.launch')

        self.gazebo_process = subprocess.Popen(['roslaunch',
                                                launch_file,
                                                'world_name:=' + world_name,
                                                'gui:=' + ("true" if gui else "false"),
                                                'verbose:=' + ("true" if verbose else "false"),
                                                ])
        time.sleep(10)  # sleep to wait until the gazebo being created

        # initialize the node for gym env
        rospy.init_node('gym', anonymous=True, log_level=rospy.FATAL)
        rospy.set_param('/use_sim_time', True)

    def set_start_goal_BARN(self, init_position, goal_position):
        """Use predefined start and goal position for BARN dataset
        """

        self.gazebo_sim = GazeboSimulation(init_position)
        self.init_position = init_position
        self.goal_position = goal_position

        # path_dir = join(self.BASE_PATH, "worlds", "BARN", "path_files")
        # world_id = int(self.world_name.split('_')[-1].split('.')[0])
        # path = np.load(join(path_dir, 'path_%d.npy' % world_id))
        # init_x, init_y = self._path_coord_to_gazebo_coord(*path[0])
        # goal_x, goal_y = self._path_coord_to_gazebo_coord(*path[-1])
        # init_y -= 1
        # goal_x -= init_x
        # goal_y -= (init_y-5) # put the goal 5 meters backward
        # self.init_position = [init_x, init_y, np.pi/2]
        # self.goal_position = [goal_x, goal_y, 0]

    def seed(self, seed):
        np.random.seed(seed)

    def reset(self):
        a = 10
        raise NotImplementedError

    def _reset_move_base(self):
        self.move_base.reset_robot_in_odom()
        self._clear_costmap()
        self.move_base.set_global_goal()

    def _clear_costmap(self):
        self.move_base.clear_costmap()
        rospy.sleep(0.1)
        self.move_base.clear_costmap()
        rospy.sleep(0.1)
        self.move_base.clear_costmap()

    def step(self, action):
        """take an action and step the environment
        """
        self._take_action(action)
        self.step_count += 1
        pos, psi = self._get_pos_psi()

        self.gazebo_sim.unpause()
        # compute observation
        obs = self._get_observation()

        # compute termination
        flip = pos.z > 0.1  # robot flip

        goal_pos = 0
        success = np.linalg.norm(goal_pos) < 0.4

        timeout = self.step_count >= self.max_step

        collided = self.gazebo_sim.get_hard_collision() and self.step_count > 1
        self.collision_count += int(collided)

        done = flip or success or timeout or self.collision_count >= self.max_collision

        # compute reward
        rew = self.slack_reward
        if done and not success:
            rew += self.failure_reward
        if success:
            rew += self.success_reward
        if collided:
            rew += self.collision_reward

        # rew += (np.linalg.norm(self.last_goal_pos) - np.linalg.norm(goal_pos)) * self.goal_reward
        # self.last_goal_pos = goal_pos

        info = dict(
            collision=self.collision_count,
            collided=collided,
            goal_position=goal_pos,
            time=self.current_time - self.start_time,
            success=success,
            world=self.world_name
        )

        if done:
            bn, nn = self.gazebo_sim.get_bad_vel_num()
            # info.update({"recovery": 1.0 * bn / nn})

        self.gazebo_sim.pause()
        return obs, rew, done, info

    def _take_action(self, action):
        current_time = rospy.get_time()
        while current_time - self.current_time < self.time_step:
            time.sleep(0.01)
            current_time = rospy.get_time()
        self.current_time = current_time

    def _get_observation(self):
        raise NotImplementedError()

    def _get_pos_psi(self):
        pose = self.gazebo_sim.get_model_state().pose
        pos = pose.position

        q1 = pose.orientation.x
        q2 = pose.orientation.y
        q3 = pose.orientation.z
        q0 = pose.orientation.w
        psi = np.arctan2(2 * (q0*q3 + q1*q2), (1 - 2*(q2**2+q3**2)))
        assert -np.pi <= psi <= np.pi, psi

        return pos, psi

    def close(self):
        # These will make sure all the ros processes being killed
        os.system("killall -9 rosmaster")
        os.system("killall -9 gzclient")
        os.system("killall -9 gzserver")
        os.system("killall -9 roscore")

    def _path_coord_to_gazebo_coord(self, x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

class JackalGazeboLaser(JackalGazebo):
    def __init__(self, laser_clip=4, **kwargs):
        super().__init__(**kwargs)
        self.laser_clip = laser_clip

        obs_dim = 720 + 1  # 720 dim laser scan + local goal (in angle)
        self.observation_space = Box(
            low=0,
            high=laser_clip,
            shape=(obs_dim,),
            dtype=np.float32
        )

    def _get_observation(self):
        # observation is the 720 dim laser scan + one local goal in angle
        laser_scan = self._get_laser_scan()
        local_goal = self._get_local_goal()

        laser_scan = (laser_scan - self.laser_clip/2.) / self.laser_clip # scale to (-0.5, 0.5)
        local_goal = local_goal / (2.0 * np.pi) # scale to (-0.5, 0.5)

        obs = np.concatenate([laser_scan, local_goal])

        return obs

    def _get_laser_scan(self):
        """Get 720 dim laser scan
        Returns:
            np.ndarray: (720,) array of laser scan 
        """
        laser_scan = self.gazebo_sim.get_laser_scan()
        laser_scan = np.array(laser_scan.ranges)
        laser_scan[laser_scan > self.laser_clip] = self.laser_clip
        return laser_scan

    def _get_local_goal(self):
        """get local goal in angle
        Returns:
            float: local goal in angle
        """
        local_goal = self.move_base.get_local_goal()[0]
        local_goal = np.array([np.arctan2(local_goal.position.y, local_goal.position.x)])
        return local_goal

    def _get_global_goal(self):
        a = 10

    def transform_goal(self, goal_pos, pos, psi):
        """ transform goal in the robot frame
        params:
            pos_1
        """
        R_r2i = np.matrix([[np.cos(psi), -np.sin(psi), pos.x], [np.sin(psi), np.cos(psi), pos.y], [0, 0, 1]])
        R_i2r = np.linalg.inv(R_r2i)
        pi = np.matrix([[goal_pos[0]], [goal_pos[1]], [1]])
        pr = np.matmul(R_i2r, pi)
        lg = np.array([pr[0,0], pr[1, 0]])
        return lg
