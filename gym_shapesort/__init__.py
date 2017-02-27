import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='BulletReacher3D-v0',
    entry_point='gym_shapesort.envs:Reacher',
    reward_threshold=1.0,
    nondeterministic = True,
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={
    	'frame_skip': 2,
    	'obs_type': 'q', 
    	'pybullet_mode': 'SHARED_MEMORY'
    }
)

register(
    id='BulletReacher3D-rgb-v0',
    entry_point='gym_shapesort.envs:Reacher',
    reward_threshold=1.0,
    nondeterministic = True,
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={
    	'frame_skip': 2,
    	'obs_type': 'rgb', 
    	'render_width': 112, 
    	'render_height': 112, 
    	'pybullet_mode': 'SHARED_MEMORY'
    }
)

register(
    id='BulletReacher3D-birgb-v0',
    entry_point='gym_shapesort.envs:Reacher',
    reward_threshold=1.0,
    nondeterministic = True,
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={
    	'frame_skip': 2,
    	'obs_type': 'birgb', 
    	'render_width': 112, 
    	'render_height': 112, 
    	'pybullet_mode': 'SHARED_MEMORY'
    }
)

register(
    id='BulletReacher3D-rgbd-v0',
    entry_point='gym_shapesort.envs:Reacher',
    reward_threshold=1.0,
    nondeterministic = True,
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={
    	'frame_skip': 2,
    	'obs_type': 'rgbd', 
    	'render_width': 112, 
    	'render_height': 112, 
    	'pybullet_mode': 'SHARED_MEMORY'
    }
)

# register(
#     id='SoccerEmptyGoal-v0',
#     entry_point='gym_soccer.envs:SoccerEmptyGoalEnv',
#     timestep_limit=1000,
#     reward_threshold=10.0,
#     nondeterministic = True,
# )

# register(
#     id='SoccerAgainstKeeper-v0',
#     entry_point='gym.envs:SoccerAgainstKeeperEnv',
#     timestep_limit=1000,
#     reward_threshold=8.0,
#     nondeterministic = True,
# )