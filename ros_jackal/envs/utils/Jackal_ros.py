import numpy as np

try:
    import rospy
    from std_msgs.msg import Bool, Float64
    from sensor_msgs.msg import LaserScan, Imu, PointCloud2
    from visualization_msgs.msg import Marker
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from tf2_msgs.msg import TFMessage
    import math
    import time
    from std_msgs.msg import Float64MultiArray

except ModuleNotFoundError:
    pass

class JackalRos:
    def __init__(self):

        self.robot_state = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'theta': 0.0,
            'velocity': 0.0,
            'angular_velocity': 0.0
        }

        self.start = False

        self.local_goal = [0, 4]
        self.global_goal = [0, 10]
        self.laser_data = None
        self.odometry_data = None
        self.cmd_vel_data = None
        self.bad_vel = 0
        self.vel_counter = 0

        self.is_colliding = False
        self.collision_count = 0
        self.collision_start_time = None
        self.last_collision_duration = None

        self._setup_subscribers()
        self._setup_publisher()

        self.last_action = np.full(20, 0.1)

        rospy.loginfo("JackalRos initialized - subscribing to ROS topics")

    def _setup_subscribers(self):

        self._laser_sub = rospy.Subscriber(
            "/front/scan", LaserScan, self._laser_callback, queue_size=1
        )

        self._odom_sub = rospy.Subscriber(
            "/odometry/filtered", Odometry, self._odometry_callback, queue_size=1
        )

        self._collision_sub = rospy.Subscriber('/collision', Bool, self._collision_callback)

        self._get_local_goal_sub = rospy.Subscriber("/local_goal", Marker, self.local_goal_callback)

        self._get_global_goal_sub = rospy.Subscriber("/global_goal", Marker, self.global_goal_callback)

    def _setup_publisher(self):
        self._dynamics_pub = rospy.Publisher('/dy_dt', Float64MultiArray, queue_size=1)
        self._params_pub = rospy.Publisher('/adp_params', Float64MultiArray, queue_size=1)

    def _collision_callback(self, msg):

        current_time = rospy.get_time()

        if msg.data:
            if not self.is_colliding:
                self.collision_count += 1
                self.collision_start_time = current_time
                self.is_colliding = True

        else:
            if self.is_colliding:
                duration = current_time - self.collision_start_time
                self.last_collision_duration = duration
                self.is_colliding = False


    def _laser_callback(self, msg):
        self.laser_data = msg

    def _odometry_callback(self, msg):
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        q0 = msg.pose.pose.orientation.w

        self.robot_state['x'] = msg.pose.pose.position.x
        self.robot_state['y'] = msg.pose.pose.position.y
        self.robot_state['z'] = msg.pose.pose.position.z

        self.robot_state['theta'] = math.atan2(
            2 * (q0 * q3 + q1 * q2),
            1 - 2 * (q2 * q2 + q3 * q3)
        )

        self.robot_state['velocity'] = msg.twist.twist.linear.x
        self.robot_state['angular_velocity'] = msg.twist.twist.angular.z

        if self.start == False:
            if self.robot_state['velocity'] >= 0.1:
                self.start = True
                self.start_time = rospy.get_time()
        else:
            if self.robot_state['velocity'] <= 0.05:
                self.bad_vel += 1

            self.vel_counter += 1

    def local_goal_callback(self, msg):
        if len(msg.points) > 0:
            local_x = msg.points[0].x
            local_y = msg.points[0].y
            self.local_goal = np.array([local_x, local_y])

    def global_goal_callback(self, msg):
        if len(msg.points) > 0:
            global_x = msg.points[0].x
            global_y = msg.points[0].y
            self.global_goal = np.array([global_x, global_y])

    def set_params(self, v):
        msg = Float64MultiArray()

        if v is None:
            msg.data = []
        else:
            # 处理不同类型的 v
            if hasattr(v, 'tolist'):  # numpy array
                msg.data = v.tolist()
            elif isinstance(v, (list, tuple)):  # 已经是列表或元组
                msg.data = list(v)
            else:  # 单个数值 (int, float)
                msg.data = [float(v)]

        self._params_pub.publish(msg)

    def set_dynamics_equation(self, action):
        msg = Float64MultiArray()

        if action is None or len(action) == 0:
            msg.data = []
        else:
            msg.data = action.tolist() if hasattr(action, 'tolist') else list(action)

        self._dynamics_pub.publish(msg)

    def get_collision(self):
        return self.is_colliding

    def get_laser_scan(self):
        return self.laser_data

    def get_robot_state(self):

        return np.array([
            self.robot_state['x'],
            self.robot_state['y'],
            self.robot_state['theta'],
            self.robot_state['velocity'],
            self.robot_state['angular_velocity']
        ])

    def get_local_goal(self):
        return self.local_goal

    def get_global_goal(self):
        return self.global_goal

    def get_cmd_vel(self):
        return self.cmd_vel_data

    def get_bad_vel(self):
        """
        Count the number of velocity command and velocity command
        that is smaller than 0.2 m/s (hard coded here, count as self.bad_vel)
        """
        return [self.bad_vel, self.vel_counter]

    def reset(self, init_params):
        self.is_colliding = False
        self.collision_count = 0
        self.collision_start_time = None
        self.last_collision_duration = None
        self.bad_vel = 0
        self.vel_counter = 0
        self.start = False
        self.start_time = 0
        self.last_action = init_params
