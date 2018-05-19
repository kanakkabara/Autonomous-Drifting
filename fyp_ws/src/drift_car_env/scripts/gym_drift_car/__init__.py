import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='CollisionGazebo-v0',
    entry_point='gym_drift_car.envs:CollisionEnv',
)

register(
    id='CollisionContinuousGazebo-v0',
    entry_point='gym_drift_car.envs:CollisionEnv',
    kwargs={'continuous' : True}
)

register(
    id='DriftCarGazebo-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
)

register(
    id='DriftCarGazeboPartialWithAngles-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    kwargs={'state_info': ["i", "j", "k", "w", "xdot", "ydot", "thetadot", "s"]}
)

register(
    id='DriftCarGazeboPartial-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    kwargs={'state_info': ["xdot", "ydot", "thetadot", "s"]}
)

register(
    id='DriftCarGazeboPartialBodyFrame-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: thetadot, xDotBody, yDotBody.
    kwargs={'state_info': ["thetadot", "xdotbodyframe", "ydotbodyframe"]}
)

register(
    id='DriftCarGazeboPartialBodyFrame4WD-v0',
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
    id='DriftCarGazeboContinuousPartialWithAngles-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: i, j, k, w, xdot, ydot, thetadot, s.
    kwargs={'continuous' : True, 'state_info': ["i", "j", "k", "w", "xdot", "ydot", "thetadot", "s"]}
)

register(
    id='DriftCarGazeboContinuousPartial-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: xdot, ydot, thetadot, s.
    kwargs={'continuous' : True, 'state_info': ["xdot", "ydot", "thetadot", "s"]}
)

register(
    id='DriftCarGazeboContinuousSpeedCost-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: xdot, ydot, thetadot, s.
    kwargs={'continuous' : True, 'state_info': ["thetadot", "s"]}
)

register(
    id='DriftCarGazeboContinuousBodyFrame-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: thetaDot, xDotBodyFrame, yDotBodyFrame
    kwargs={'continuous' : True, 'state_info': ["thetadot", "xdotbodyframe", "ydotbodyframe"]}
)

register(
    id='DriftCarGazeboContinuousBodyFrame4WD-v0',
    entry_point='gym_drift_car.envs:GazeboEnv',
    # state: thetaDot, xDotBodyFrame, yDotBodyFrame
    kwargs={'continuous' : True, 'four_wheel_drive': True, 'state_info': ["thetadot", "xdotbodyframe", "ydotbodyframe"]}
)