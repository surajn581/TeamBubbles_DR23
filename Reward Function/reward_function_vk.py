import math
import logging
import numpy as np
from scipy import signal

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

def distance(p1, p2):
    """ Euclidean distance between two points """ 
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def angle(p1, p2):
    """
    """
    dy = p2[1]-p1[1]
    dx = p2[0]-p1[0]
    return math.degrees(math.atan2(dy,dx))

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
    starting = (params["x"], params["y"])

    waypoints = list(starting) + waypoints

    increased_precision = up_sample(waypoints, scaling_factor)
    increased_precision.pop(0)
    return waypoints

def target_angle(params):
    wp = get_waypoints(params, 2)
    return angle(wp[0], wp[1])    

def is_a_turn_coming_up( params, n_points, angle_threshold ):
    wp = get_waypoints(params, 2)
    angles = [ angle( wp[i], wp[i+1] ) for i in range( min( n_points-1, len(wp) ) ) ]
    diff_angles = [ abs(angles[i] - angles[i+1]) for i in range(len(angles) - 1) ]
    return not all([ diff < angle_threshold for diff in diff_angles ])

def is_higher_speed_favorable(params):
    """ no high difference in heading  """
    return 10 * params["speed"] * (-0.01 if is_a_turn_coming_up( params, n_points=15, angle_threshold=4 ) else 1)
     
def is_steps_favorable(params):
    return 1 * 100 / params["steps"]

def get_target_steering_degree(params):
    tx, ty = get_waypoints(params,2)[0]
    car_x = params['x']
    car_y = params['y']
    dx = tx-car_x
    dy = ty-car_y
    heading = params['heading']
    target_angle = angle((car_x, car_y), (tx, ty))
    steering_angle = target_angle - heading
    return steering_angle % 360

def is_progress_favorable(params):
    return params["progress"] / 10

def off_center_penalty( params ):
    ''' function to encourage the model to stay close to the track center when there are no curves coming up'''
    #TODO check how we can improve this logic
    threshold = params['track_width']*0.1
    distance_from_center = params[ 'distance_from_center' ]
    path_is_straight = not is_a_turn_coming_up( params, n_points=15, angle_threshold=5 )    
    if path_is_straight:
        # if path is straight then greater distance from center will be penalised when the distance is greater than threshold
        # and if the distance from center is less than threshold, a reward of 10 will be given
        return -5*distance_from_center if distance_from_center>threshold else 10
    return 0

def score_steer_to_point_ahead(params):
    best_stearing_angle = get_target_steering_degree(params)
    steering_angle = params['steering_angle']
    error = (steering_angle - best_stearing_angle) / 30.0   # keeping 30 degrees as the threshold for error in the angle
    score = 1.0 - error**2
    return is_steps_favorable(params) * is_progress_favorable(params) * is_higher_speed_favorable(params) + score + off_center_penalty(params)

def calculate_reward(params):
    if params["is_offtrack"] or params["is_crashed"]:
        return -100.0
    return float(score_steer_to_point_ahead(params))

def reward_function(params):
    try:
        return float(calculate_reward(params))
    except Exception as ex:
        logging.error('[EXCEPTION444] %s', ex)
        return float(0)