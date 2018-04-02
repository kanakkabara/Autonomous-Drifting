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
    # state: xdot, ydot, thetadot, s.
    kwargs={'state_info': {'state_size': 4, 'include_tangential_speed': True}}
)

register(
    id='DriftCarGazeboPartialWithAnglesEnv-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: i, j, k, w, xdot, ydot, thetadot, s.
    kwargs={'state_info': {'state_size': 8, 'include_tangential_speed': True, 'include_theta': True}}
)

register(
    id='DriftCarGazeboPartialBodyFrame-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: thetadot, xDotBody, yDotBody.
    kwargs={'four_wheel_drive': True, 'state_info': {'state_size': 3, 'include_body_frame_velocity': True}}
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
    kwargs={'continuous' : True, 'state_info': {'state_size': 4, 'include_tangential_speed': True}}
)

register(
    id='DriftCarGazeboContinuousPartialWithAngles-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: i, j, k, w, xdot, ydot, thetadot, s.
    kwargs={'continuous' : True, 'state_info': {'state_size': 8, 'include_tangential_speed': True, 'include_theta': True}}
)
