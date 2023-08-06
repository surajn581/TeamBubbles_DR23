"""
params = {
    "all_wheels_on_track": Boolean,        # flag to indicate if the agent is on the track
    "x": float,                            # agent's x-coordinate in meters
    "y": float,                            # agent's y-coordinate in meters
    "closest_objects": [int, int],         # zero-based indices of the two closest objects to the agent's current position of (x, y).
    "closest_waypoints": [int, int],       # indices of the two nearest waypoints.
    "distance_from_center": float,         # distance in meters from the track center 
    "is_crashed": Boolean,                 # Boolean flag to indicate whether the agent has crashed.
    "is_left_of_center": Boolean,          # Flag to indicate if the agent is on the left side to the track center or not. 
    "is_offtrack": Boolean,                # Boolean flag to indicate whether the agent has gone off track.
    "is_reversed": Boolean,                # flag to indicate if the agent is driving clockwise (True) or counter clockwise (False).
    "heading": float,                      # agent's yaw in degrees
    "objects_distance": [float, ],         # list of the objects' distances in meters between 0 and track_length in relation to the starting line.
    "objects_heading": [float, ],          # list of the objects' headings in degrees between -180 and 180.
    "objects_left_of_center": [Boolean, ], # list of Boolean flags indicating whether elements' objects are left of the center (True) or not (False).
    "objects_location": [(float, float),], # list of object locations [(x,y), ...].
    "objects_speed": [float, ],            # list of the objects' speeds in meters per second.
    "progress": float,                     # percentage of track completed
    "speed": float,                        # agent's speed in meters per second (m/s)
    "steering_angle": float,               # agent's steering angle in degrees
    "steps": int,                          # number steps completed
    "track_length": float,                 # track length in meters.
    "track_width": float,                  # width of the track
    "waypoints": [(float, float), ]        # list of (x,y) as milestones along the track center

}
"""
import math
import numpy as np
from scipy import signal

def distance(p1, p2):
    """ Euclidean distance between two points """ 
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def angle(p1, p2):
    """
    """
    dy = p2[1]-p1[1]
    dx = p2[0]-p1[0]
    return math.degrees(math.atan2(dy,dx))

def normalize_angle_to_360(angle):
    if angle < 0:
        return 360 + angle
    return angle

def up_sample(waypoints, factor):
    """
    Adds extra waypoints in between provided waypoints
    :param waypoints:
    :param factor: integer. E.g. 3 means that the resulting list has 3 times as many points.
    :return:
    """
    return list( signal.resample(np.array(waypoints), len(waypoints) * factor) )

def get_waypoints(params, scaling_factor):
    """ Way-points """
    if params['is_reversed']: # driving clock wise.
        waypoints = list(reversed(params['waypoints']))
    else: # driving counter clock wise.
        waypoints = params['waypoints']    
    waypoints = waypoints[params["closest_waypoints"][1]: ]
    # starting = (params["x"], params["y"])

    # waypoints = list(starting) + waypoints

    # increased_precision = up_sample(waypoints, scaling_factor)
    # increased_precision.pop(0)
    return waypoints

def target_angle(params):
    wp = get_waypoints(params, 2)
    return angle(wp[0], wp[1])    

def is_a_turn_coming_up( params, n_points, angle_threshold ):
    wp = get_waypoints(params, 2)
    angles = [ angle( wp[i], wp[i+1] ) for i in range( min( n_points-1, len(wp)-1 ) ) ]
    angles = [ normalize_angle_to_360(angle) for angle in angles ]
    diff_angles = [ abs(angles[i] - angles[i+1]) for i in range(len(angles) - 1) ]
    return not all([ diff < angle_threshold for diff in diff_angles ])

def is_higher_speed_favorable(params):
    """ no high difference in heading  """
    # speed range 2-4 > 0 - 6
    return 1.5 * params["speed"] * (0 if is_a_turn_coming_up( params, n_points=25, angle_threshold=4 ) else 1)
     
def is_steps_favorable(params):
    # if number of steps range (1-150) > (0.66 - 100)
    # if number of steps range (1-900) > (0.11 - 100)
    if params['progress'] != 100:
        return float( 50 / params["steps"] )
    return float( 100 / params["steps"] )

def get_target_heading_degree_reward(params):
    # reward range 0-5
    tx, ty = get_waypoints(params,2)[0]
    car_x, car_y = params['x'], params['y']
    heading = params['heading']
    target_angle = angle((car_x, car_y), (tx, ty))
    diff = abs( target_angle - heading )
    diff = 360-diff if diff>180 else diff
    threshold = 5
    if diff > threshold:
        return -5
    return 5

def is_steering_too_much(params):
    abs_steering = abs(params['steering_angle'])
    reward = 1.0
    ABS_STEERING_THRESHOLD = 20.0
    if abs_steering > ABS_STEERING_THRESHOLD:
        reward *= 0.5
    return float(reward)

def is_progress_favorable(params):
    # progress range is 1-100 > reward range is 0.1 - 10
    return params["progress"] / 10

def off_center_penalty( params ):
    ''' function to encourage the model to stay close to the track center when there are no curves coming up'''
    #TODO check how we can improve this logic
    threshold = params['track_width']*0.1
    distance_from_center = params[ 'distance_from_center' ]
    path_is_straight = not is_a_turn_coming_up( params, n_points=25, angle_threshold=5 )    
    threshold = params['track_width'] * ( 0.1 if path_is_straight else 0.3 )
    # if path is straight then greater distance from center will be penalised when the distance is greater than threshold
    # and if the distance from center is less than threshold, a reward of 10 will be given
    return -20*distance_from_center if distance_from_center>threshold else 10

def score_steer_to_point_ahead(params):
    heading_reward      = get_target_heading_degree_reward(params)
    steps_reward        = is_steps_favorable(params)
    progress_reward     = is_progress_favorable(params)
    speed_reward        = is_higher_speed_favorable(params)
    track_center_reward = off_center_penalty(params)
    steering_reward     = is_steering_too_much(params)
    reward              = (speed_reward) * (heading_reward) * (steering_reward) + steps_reward*progress_reward + (3*track_center_reward)
    return reward

def calculate_reward(params):
    if params["is_offtrack"] or params["is_crashed"]:
        return -500.0
    return float(score_steer_to_point_ahead(params))

def reward_function(params):
    return float(calculate_reward(params))