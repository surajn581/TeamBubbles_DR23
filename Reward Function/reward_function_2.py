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

def distance_bw_points(p1, p2):
    """ Euclidean distance between two points """ 
    return ((p1[0] - p2[0]) * 2 + (p1[1] - p2[1]) * 2) ** 0.5

def angle_bw_points(p1, p2):
    return math.degrees(math.atan2(p2[1] - p1[1],p2[0] - p1[0]))

def normalize_angle_to_360(angle):
    if angle < 0:
        return 360 + angle
    return angle

def get_future_heading(params):
    heading = normalize_angle_to_360(params["heading"]) # From X - axis, it is the counter clockwiese angle (-180, 180)
    steering_angle = params["steering_angle"]

    return normalize_angle_to_360(heading + steering_angle)

def get_speed_reward(params):
    waypoints = params["waypoints"]
    closest_waypoints = params["closest_waypoints"]
    next_waypoint_index = closest_waypoints[1]
    next_3_waypoints = waypoints[next_waypoint_index: min(len(waypoints), next_waypoint_index + 3)] 
    current_waypoint = (params["x"], params["y"])
    waypoint_set = [current_waypoint] + next_3_waypoints
    # If low deviation between current + next 5 waypoints, it is good to have more speed
    target_heading_bw_points = [ angle_bw_points(waypoint_set[i], waypoint_set[i+1]) for i in range(len(waypoint_set) - 1 ) ]
    target_heading_bw_points = [ normalize_angle_to_360(angle) for angle in target_heading_bw_points ]

    if all(normalize_angle_to_360(target_heading_bw_points[index + 1] - target_heading_bw_points[index]) < 4 for index in range(len(target_heading_bw_points)-1)):
        return params["speed"] * 100

    # We still want to reward high speeds a bit but not as much as a straight line, so steep curves can be accomodated
    return params["speed"]


def get_heading_reward(params):
    waypoints = params["waypoints"]
    closest_waypoints = params["closest_waypoints"]
    # TODO VK Take a range across track lenght for better angles. but we dont need that here. We just want to see reverse i.e. > 70 would work
    next_waypoint = waypoints[closest_waypoints[1]]
    current_waypoint = (params["x"], params["y"])
    target_heading = normalize_angle_to_360(angle_bw_points(current_waypoint, next_waypoint))
    future_angle = get_future_heading(params)
    difference =  abs(future_angle - target_heading)

    if difference <= 1:
        return 200

    if difference <= 5:
        return 100

    return 80 - difference

def is_opposite_direction(params):
    future_angle = get_future_heading(params)

    waypoints = params["waypoints"]
    closest_waypoints = params["closest_waypoints"]
    # TODO VK Take a range across track lenght for better angles. but we dont need that here. We just want to see reverse i.e. > 70 would work
    next_waypoint = waypoints[closest_waypoints[1]]
    current_waypoint = (params["x"], params["y"])
    target_heading = normalize_angle_to_360(angle_bw_points(current_waypoint, next_waypoint))
    # We will need to fix this on zig zag
    if abs(future_angle - target_heading) > 70:
        return 0
    return 1

def is_off_track(params):
    if(params["is_offtrack"] or params["is_crashed"]):
        return -10

    if not params["all_wheels_on_track"]:
        return 0.9

    return 1

def get_steps_reward(params):
    return 1. / params["steps"]

def get_progress_reward(params):
    return params["progress"]

def reward_function(params):
    # should be encouraged to complete
    if params["progress"] == 100:
        return 100 * 10 / params["steps"]

    return is_off_track(params) * is_opposite_direction(params) * [ get_speed_reward(params) + get_heading_reward(params)]