from gym.spaces import Box
import numpy as np

try:
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray
except ModuleNotFoundError:
    pass

from envs import JackalBase, JackalLaser, FidelityEquation_function,  FidelityEquation_continues, Visualization

class AdaptiveDynamicsPlanning_Continues(JackalLaser, FidelityEquation_continues,  Visualization):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

class AdaptiveDynamicsPlanning_Functional(JackalLaser, FidelityEquation_function, Visualization):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)