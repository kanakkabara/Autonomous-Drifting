import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='DriftCarGazeboEnv-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
)

register(
    id='DriftCarGazeboPartialEnv-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    kwargs={'partial': True}
)

register(
    id='DriftCarGazeboContinuous-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    kwargs={'continuous' : True}
)

register(
    id='DriftCarGazeboContinuousPartial-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    kwargs={'continuous' : True, 'partial': True}
)
#register(
#    id='DriftCarEnv-v0',
#    entry_point='gym_foo.envs:DriftCarEnv',
#)
