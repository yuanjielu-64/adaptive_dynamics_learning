from gym.envs.registration import register

register(
    id="adaptive_dynamics_planning_v0",
    entry_point="envs.adaptive_dynamics_envs:AdaptiveDynamicsPlanning"
)