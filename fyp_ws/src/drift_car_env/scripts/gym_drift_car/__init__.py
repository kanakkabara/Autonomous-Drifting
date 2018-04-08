import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='DriftCarGazebo-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
)

register(
    id='DriftCarGazeboPartial-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    kwargs={'state_info': ["xdot", "ydot", "thetadot", "s"]}
)

register(
    id='DriftCarGazeboPartialWithAngles-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    kwargs={'state_info': ["i", "j", "k", "w", "xdot", "ydot", "thetadot", "s"]}
)

register(
    id='DriftCarGazeboPartialBodyFrame-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: thetadot, xDotBody, yDotBody.
    kwargs={'four_wheel_drive': True, 'state_info': ["thetadot", "xdotbodyframe", "ydotbodyframe"]}
)

register(
    id='DriftCarGazeboContinuous-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: x, y, i, j, k, w, xDotWorldFrame, yDotWorldFrame, thetaDot, s, xDotBodyFrame, yDotBodyFrame
    kwargs={'continuous' : True}
)

register(
    id='DriftCarGazeboContinuous4WD-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: x, y, i, j, k, w, xDotWorldFrame, yDotWorldFrame, thetaDot, s, xDotBodyFrame, yDotBodyFrame
    kwargs={'continuous' : True, 'four_wheel_drive': True}
)

register(
    id='DriftCarGazeboContinuousPartial-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: xdot, ydot, thetadot, s.
    kwargs={'continuous' : True, 'state_info': ["xdot", "ydot", "thetadot", "s"]}
)

register(
    id='DriftCarGazeboContinuousPartialWithAngles-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: i, j, k, w, xdot, ydot, thetadot, s.
    kwargs={'continuous' : True, 'state_info': ["i", "j", "k", "w", "xdot", "ydot", "thetadot", "s"]}
)