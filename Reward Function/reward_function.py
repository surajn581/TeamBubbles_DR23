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

    logger.info("Length of waypoints: ", len(driving_dir_waypoints))

    closest_waypoint = params["closest_waypoints"]  # 0th: prev wp, 1st: next wp
    
    next_closest = closest_waypoint[1]
    return driving_dir_waypoints[next_closest]

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
    # TODO make this dynamic based on the track length, number of way points, overall shape of the track
    TOTAL_NUM_STEPS = 220
    ERROR_THRESHOLD = 0.25
    MAX_SPEED = 3.7
    # Initialize the reward with typical value
    reward = 0

    # Give Reward for progress only if error is < the Error treshold
    # TODO we need a better way to implement lines 123-130
    if progress > (steps / TOTAL_NUM_STEPS) * 100 and error < ERROR_THRESHOLD:
        reward += ( progress/100 ) * (ERROR_THRESHOLD-error) * int( steps%100==0 )

    if progress == 100:
        reward -= error+abs(params["speed"] - MAX_SPEED)

    if (steps % 100) == 0 and progress == 100 and error < 0.0625 and params["speed"] == MAX_SPEED:
        reward = reward * 2 if reward > 0 else reward+5
        
    #scaling the reward by the progress to encourage the model to prioritize progress,
    #we can even use exponential scaling for this.    
    reward *= (progress/100)
    return float(reward)


def score_steer_to_point_ahead(params):
    best_stearing_angle = get_target_steering_degree(params)
    steering_angle = params['steering_angle']

    error = (steering_angle - best_stearing_angle) / 30.0  # keeping 30 degrees as the threshold for error in the angle
    error = error**2 # squaring the error to get rid of the negative as well as amplifying the error
    score = 1.0 - error
    return score + get_progress_score(params, error)

def reward_function(params):
    if params["is_offtrack"] or params["is_crashed"]:
        return -1 #not sure if the reward can be negative or not, but I think negative reward makes sense
    return float(score_steer_to_point_ahead(params))
