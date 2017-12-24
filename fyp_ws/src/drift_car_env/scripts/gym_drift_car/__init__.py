import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='DriftCarGazeboEnv-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
)
#register(
#    id='DriftCarEnv-v0',
#    entry_point='gym_foo.envs:DriftCarEnv',
#)
