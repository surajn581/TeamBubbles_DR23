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


def dist(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5



def rect(r, theta):
    """
    theta in degrees
    returns tuple; (float, float); (x,y)
    """

    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y



def polar(x, y):
    """
    returns r, theta(degrees)
    """

    r = (x ** 2 + y ** 2) ** .5
    theta = math.degrees(math.atan2(y,x))
    return r, theta


def angle_mod_360(angle):
    """
    Maps an angle to the interval -180, +180.
    Examples:
    angle_mod_360(362) == 2
    angle_mod_360(270) == -90
    :param angle: angle in degree
    :return: angle in degree. Between -180 and +180
    """

    n = math.floor(angle/360.0)

    angle_between_0_and_360 = angle - n*360.0

    if angle_between_0_and_360 <= 180.0:
        return angle_between_0_and_360
    else:
        return angle_between_0_and_360 - 360


def get_waypoints_ordered_in_driving_direction(params):
    # waypoints are always provided in counter clock wise order
    if params['is_reversed']: # driving clock wise.
        return list(reversed(params['waypoints']))
    else: # driving counter clock wise.
        return params['waypoints']


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


def get_target_point(params):

    #We took next waypoints se aage ka
    driving_dir_waypoints = get_waypoints_ordered_in_driving_direction(params)
    closest_waypoint = params["closest_waypoints"]  # 0th: prev wp, 1st: next wp
    
    next_closest = closest_waypoint[1]
    return driving_dir_waypoints[next_closest]
    
#    index_wp = driving_dir_waypoints.index(next_closest)
#
#    input_up_sample = driving_dir_waypoints[index_wp:]
#    
#    
#    waypoints = up_sample(input_up_sample, 1)
#
#    car = [params['x'], params['y']]

#    distances = [dist(p, car) for p in waypoints]
#    min_dist = min(distances)
#    i_closest = distances.index(min_dist)
#
#    n = len(waypoints)
#
#    waypoints_starting_with_closest = [waypoints[(i+i_closest) % n] for i in range(n)]

#    r = params['track_width'] * 0.9
#
#    is_inside = [dist(p, car) < r for p in waypoints]
#    i_first_outside = is_inside.index(False)

#    if i_first_outside < 0:  # this can only happen if we choose r as big as the entire track
#        return waypoints[i_closest]

#    return waypoints_starting_with_closest[i_first_outside]



def get_target_steering_degree(params):
    tx, ty = get_target_point(params)
    car_x = params['x']
    car_y = params['y']
    dx = tx-car_x
    dy = ty-car_y
    heading = params['heading']

    _, target_angle = polar(dx, dy)

    steering_angle = target_angle - heading

    return angle_mod_360(steering_angle)


def get_progress_score(params, error):
    steps = params['steps']
    progress = params['progress']

    # Total num of steps we want the car to finish the lap, it will vary depends on the track length
    TOTAL_NUM_STEPS = 220
    ERROR_THRESHOLD = 0.5
    MAX_SPEED = 3.7
    MIN_SPEED = 1.5
    # Initialize the reward with typical value
    reward = 0

    # Give Reward for progress only if error is < the Error treshold
    
    # Here if ERROR_THRESHOLD/ < error < ERROR_THRESHOLD then only give  (progress / 100) as reward
    if error < ERROR_THRESHOLD and error > ERROR_THRESHOLD/2:
        # Give additional reward if the car pass every 100 steps faster than expected
        if (steps % 50) == 0 and progress > (steps / TOTAL_NUM_STEPS) * 100 :
            reward += (progress / 100)

    # Here if ERROR_THRESHOLD/4 < error < ERROR_THRESHOLD/2 then only give  (progress / 100) + (progress / 200) as reward
    elif error <= ERROR_THRESHOLD/2 and error >= ERROR_THRESHOLD/4:
        # Give additional reward if the car pass every 100 steps faster than expected
        if (steps % 50) == 0 and progress > (steps / TOTAL_NUM_STEPS) * 100 :
            reward += (progress / 100) + (progress / 200)


    
    if progress == 100 and error < 0.15 and ((params["speed"] - MAX_SPEED) < 0.3 ):
        reward = 2*(1 +(progress / 100)) + 5
    elif progress == 100 and error < 0.25 and ((params["speed"] - MAX_SPEED) < 0.3 ):
        reward = 2*(1 +(progress / 100)) + 4
    elif progress == 100 and error < 0.25 and ((params["speed"] - MAX_SPEED) < 0.6 ):
        reward = 2*(1 +(progress / 100)) + 3
    elif progress == 100 and error < 0.25 and ((params["speed"] - MAX_SPEED) < 1.5 ):
        reward = 2*(1 +(progress / 100)) + 2
    elif progress == 100 and error < 0.25 and ((params["speed"] - MAX_SPEED) < 2 ):
        reward = 2*(1 +(progress / 100))
    
    elif progress == 100 and error > 0.25 and error < 0.65 and  ((params["speed"] - MAX_SPEED) < 0.6 ):
        reward = 3 +(progress / 100)
    elif progress == 100 and error > 0.25 and error < 0.65 and  ((params["speed"] - MAX_SPEED) < 1.5 ):
        reward = 2 +(progress / 100)
    elif progress == 100 and error > 0.25 and error < 0.65 and  ((params["speed"] - MAX_SPEED) < 2 ):
        reward = 1 +(progress / 100)
    

    if (steps % 100) == 0 and progress == 100 and error < 0.25 and params["speed"] == MAX_SPEED:
        reward = reward * 2 
        
    return float(reward)


def score_steer_to_point_ahead(params):
    best_stearing_angle = get_target_steering_degree(params)
    steering_angle = params['steering_angle']

    error = (steering_angle - best_stearing_angle) / 60.0  # 60 degree is already really bad

    score = 1.0 - abs(error)

    return max(score + get_progress_score(params, abs(error)), 0.01)  # optimizer is rumored to struggle with negative numbers and numbers too close to zero


def reward_function(params):
    if params["is_offtrack"] or params["is_crashed"]:
        return 0.0001
    return float(score_steer_to_point_ahead(params))
