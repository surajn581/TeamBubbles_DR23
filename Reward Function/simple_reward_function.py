import math
def reward_function(params):
    # Read input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    steps = params['steps']
    progress = params['progress']
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    steering = abs(params['steering_angle'])
    speed = params['speed']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    is_reversed = params['is_reversed']
    heading = params['heading']

    # Reward/Penalty weights
    progress_reward = 3.0
    speed_reward = 5.0
    centering_reward = 2.0
    steering_penalty = 2.5
    off_track_penalty = 15.0
    heading_penalty = 1.5
    reverse_penalty = 5.0

    # Calculate progress
    if is_reversed:
        progress = -progress

    # Reward for making progress
    reward = progress_reward * progress

    # Penalize steering too much
    reward -= steering_penalty * steering

    # Penalize going in the wrong direction (if applicable)
    if is_reversed and progress > 0:
        reward -= reverse_penalty

    # Penalize being off track
    if not all_wheels_on_track:
        reward -= off_track_penalty

    # Calculate distance from center
    track_center = track_width / 2.0
    distance_from_center_normalized = distance_from_center / track_center

    # Reward for staying close to the center
    reward += centering_reward * (1.0 - distance_from_center_normalized)

    # Calculate speed reward
    if speed < 0.8:  # Penalize low speed
        reward *= 0.7
    else:
        reward += speed_reward * speed

    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    DIRECTION_THRESHOLD = 10.0
    if direction_diff > DIRECTION_THRESHOLD:
        reward *= 0.5


    # Penalize incorrect heading (if applicable)
    heading_error = abs(heading - waypoints[closest_waypoints[1]][2])
    reward -= heading_penalty * heading_error

    return float(reward)
