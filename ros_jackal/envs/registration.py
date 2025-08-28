from gym.envs.registration import register

register(
    id="adaptive_dynamics_planning_continues_v0",
    entry_point="envs.adaptive_dynamics_envs:AdaptiveDynamicsPlanning_Continues"
)

register(
    id="adaptive_dynamics_planning_functional_v0",
    entry_point="envs.adaptive_dynamics_envs:AdaptiveDynamicsPlanning_Functional"
)