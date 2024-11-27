from gym.envs.registration import register

register(
    id='BlackBirdGazebo-v0',
    entry_point='blackbird_gz:BlackbirdGazebo',
    max_episode_steps=1000000,
)

register(
    id='BlackBirdBalance-v0',
    entry_point='blackbird_gz:BlackbirdBalance',
    max_episode_steps=1000000,
)