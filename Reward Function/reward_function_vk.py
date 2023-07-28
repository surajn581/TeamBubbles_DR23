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
    return ((p1[0] - p2[0]) * 2 + (p1[1] - p2[1]) * 2) ** 0.5

def angle(p):
    """
    """
    return math.degrees(math.atan2(p[1],p[0]))

def up_sample(waypoints, factor):
    """
    Adds extra waypoints in between provided waypoints
    :param waypoints:
    :param factor: integer. E.g. 3 means that the resulting list has 3 times as many points.
    :return:
    """
    p = waypoints
    n = len(p)

    return [[i / factor * p[(j+1) % n][0] + (1 - i / factor) * p[j][0],
             i / factor * p[(j+1) % n][1] + (1 - i / factor) * p[j][1]] for j in range(n) for i in range(factor)]

def get_waypoints(params, scaling_factor):
    """ Way-points """
    if params['is_reversed']: # driving clock wise.
        waypoints = list(reversed(params['waypoints']))
    else: # driving counter clock wise.
        waypoints = params['waypoints']
    
    waypoints = waypoints[params["closest_waypoints"][1]: ]

    starting = (params["x"], params["y"])

    waypoints = starting + waypoints

    increased_precision = up_sample(waypoints, scaling_factor)
    increased_precision.pop(0)

    return increased_precision    

def target_angle(params):
    wp = get_waypoints(params, 2)
    return angle(wp[0])    

def is_higher_speed_favorable(params):
    """ no high difference in heading  """
    wp = get_waypoints(params, 2)

    diff_threshold = 5
    points_threshold = 10

    angles = [angle(p) for p in wp[0:points_threshold]] 

    if all(abs(angles[i] - angles[i+1]) <= diff_threshold for i in range(len(angles) - 1)):
        return 100 * params["speed"]

    # More deviation, less favorable higher speeds
    return max(10, 100 - sum(abs(angles[i] - angles[i+1]) for i in range(len(angles) - 1))) * params["speed"]
     
def is_steps_favorable(params):
    return 1 * 100 / params["steps"]

def is_progress_favorable(params):
    return params["progress"] / 10

def reward_function(params):
    if params["is_offtrack"] or params["is_crashed"]:
        return -10
    return float(is_progress_favorable(params) * is_higher_speed_favorable(params) * is_steps_favorable(params))