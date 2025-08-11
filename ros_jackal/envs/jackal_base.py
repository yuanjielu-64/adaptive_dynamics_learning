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

from .utils import GazeboSimulation, MoveBase, JackalRos


class JackalBase(gym.Env):
    def __init__(
            self,
            world_name="jackal_world.world",
            gui=False,
            init_position=None,
            goal_position=None,
            max_step=100,
            time_step=1,
            slack_reward=-1,
            failure_reward=-50,
            success_reward=0,
            obstacle_reward=0,
            goal_reward=1,
            max_collision=10000,
            verbose=True,
    ):
        """Base RL env that initialize jackal simulation in Gazebo
        """
        super().__init__()
        # config
        if init_position is None:
            init_position = [-2, 3, 0]
        if goal_position is None:
            goal_position = [10, 0, 0]

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
        self.obstacle_reward = obstacle_reward
        self.goal_reward = goal_reward
        self.max_collision = max_collision

        self.launch_gazebo(world_name=self.world_name, gui=self.gui, verbose=self.verbose)
        self.set_start_goal_BARN(init_position, goal_position)
        self.launch_move_base(goal_position=self.global_goal,
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
        self.move_base_process = subprocess.Popen(
            ['roslaunch', launch_file, 'base_local_planner:=' + base_local_planner])
        self.move_base = MoveBase(goal_position=goal_position, base_local_planner=base_local_planner)

    def launch_gazebo(self, world_name, gui, verbose):
        # launch gazebo
        rospy.logwarn(">>>>>>>>>>>>>>>>>> Load world: %s <<<<<<<<<<<<<<<<<<" % (world_name))

        # ros_package_path = os.environ.get('ROS_PACKAGE_PATH', 'Not set')
        # print(f"ROS_PACKAGE_PATH: {ros_package_path}")

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
        self.jackal_ros = JackalRos()
        self.start_position = init_position
        self.global_goal = goal_position
        self.local_goal = [0, 0, 0]

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
        """reset the environment without setting the goal
        set_goal is replaced with make_plan
        """
        self.step_count = 0

        self.gazebo_sim.reset()
        self.jackal_ros.reset()
        self.start_time = self.current_time = self.jackal_ros.start_time

        # -----------------------
        self.gazebo_sim.unpause()
        self._reset_move_base()
        self.jackal_ros.set_dynamics_equation([])
        obs = self._get_observation()
        self.gazebo_sim.pause()
        # -----------------------

        self._reset_reward()

        return obs

    def step(self, action):
        """take an action and step the environment
        """

        self._take_action(action)
        self.step_count += 1

        obs = self._get_observation()
        pos = self._get_robot_state()
        rew = self._get_reward()
        done = self._get_done()
        info = self._get_info()

        self.traj_pos.append((pos[0], pos[1]))

        return obs, rew, done, info

    def _reset_reward(self):
        self.traj_pos = []
        self.collision_count = 0
        self.smoothness = 0
        
        # Initialize distance tracking for distance progress reward
        robot_pos = self.jackal_ros.get_robot_state()
        self.last_distance = self._compute_distance(
            [robot_pos[0], robot_pos[1]],
            self.global_goal
        )

    def _get_reward(self):

        if self.jackal_ros.get_collision():
            return self.failure_reward
        elif self.step_count >= self.max_step:
            return self.failure_reward

        if self._get_success():
            time_efficiency = (self.max_step - self.step_count) / self.max_step
            rewards = self.success_reward + time_efficiency * 50
        else:
            rewards = self.slack_reward

        # obstacles
        laser_scan = np.array(self.jackal_ros.laser_data.ranges)
        d = np.mean(sorted(laser_scan)[:10])
        if d < 0.15:
            rewards  += self.obstacle_reward / (d + 0.1)

        # distance progress reward
        robot_pos = self.jackal_ros.get_robot_state()
        current_distance = self._compute_distance(
            [robot_pos[0], robot_pos[1]],
            self.global_goal
        )

        distance_progress = self.last_distance - current_distance
        rewards += distance_progress * 25
        self.last_distance = current_distance

        smoothness = self._compute_angle(len(self.traj_pos) - 1)
        self.smoothness += smoothness

        return rewards

    def _get_done(self):
        success = self._get_success()
        done = success or self.step_count >= self.max_step or self._get_flip_status()
        return done


    def _get_success(self):
        robot_position = [self.jackal_ros.get_robot_state()[0], self.jackal_ros.get_robot_state()[1]]
        if robot_position[1] > self.global_goal[1]:
            return True
        if self._compute_distance(robot_position, self.global_goal) <= 2:
            return True
        return False


    def _get_info(self):
        bn, nn = self.jackal_ros.get_bad_vel()
        self.collision_count += self.jackal_ros.get_collision()
        return dict(
            world=self.world_name,
            time=rospy.get_time() - self.jackal_ros.start_time,
            collision=self.collision_count,
            recovery=1.0 * (bn + 0.0001) / (nn + 0.0001),
            smoothness=self.smoothness
        )


    def _compute_distance(self, p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


    def _compute_angle(self, idx):
        def dis(x1, y1, x2, y2):
            return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        assert self.traj_pos is not None
        if len(self.traj_pos) > 2:
            x1, y1 = self.traj_pos[idx - 2]
            x2, y2 = self.traj_pos[idx - 1]
            x3, y3 = self.traj_pos[idx]
            a = - np.arccos(((x2 - x1) * (x3 - x2) + (y2 - y1) * (y3 - y2)) / dis(x1, y1, x2, y2) / dis(x2, y2, x3, y3))
        else:
            a = 0
        return a


    def _get_flip_status(self):
        robot_position = self.jackal_ros.robot_state['z']
        return robot_position > 0.1


    def _take_action(self, action):
        raise NotImplementedError()


    def _get_observation(self):
        raise NotImplementedError()


    def _get_robot_state(self):
        raise NotImplementedError()


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


    def close(self):
        # These will make sure all the ros processes being killed
        os.system("killall -9 rosmaster")
        os.system("killall -9 gzclient")
        os.system("killall -9 gzserver")
        os.system("killall -9 roscore")


    def _get_pos_psi(self):
        pose = self.gazebo_sim.get_model_state().pose
        pos = pose.position

        q1 = pose.orientation.x
        q2 = pose.orientation.y
        q3 = pose.orientation.z
        q0 = pose.orientation.w
        psi = np.arctan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2 ** 2 + q3 ** 2)))
        assert -np.pi <= psi <= np.pi, psi

        return pos, psi


    def _path_coord_to_gazebo_coord(self, x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)
