import numpy as np
'''
goal_width = 0.4 
ball_radius = 0.0215 
penalty_width = 0.7 
goal_depth = 0.1 
rbt_kicker_width = -1.0 
penalty_length = 0.15 
length = 1.5 
width = 1.3 
rbt_distance_center_kicker = -1.0 
rbt_kicker_thickness = -1.0 
rbt_wheel_radius = 0.02 # RC 0.026
rbt_motor_max_rpm = 715.0 # RC 440
rbt_wheel0_angle = 90.0 
rbt_wheel1_angle = 270.0 
rbt_wheel2_angle = -1.0 
rbt_wheel3_angle = -1.0 
rbt_radius = 0.0375
max_wheel_rad_s = (rbt_motor_max_rpm / 60) * 2 * np.pi
max_v = max_wheel_rad_s * rbt_wheel_radius
max_w = np.rad2deg(max_v / 0.04)
max_pos = max(width / 2, (length / 2) + penalty_length)
NORM_BOUNDS = 1.2
'''


"""
goal_width = 0.4 
ball_radius = 0.0215 
penalty_width = 0.7 
goal_depth = 0.1 
rbt_kicker_width = -1.0 
penalty_length = 0.15 
length = 1.5 
width = 1.3 
rbt_distance_center_kicker = -1.0 
rbt_kicker_thickness = -1.0 
rbt_wheel_radius = 0.02 # RC 0.026
rbt_motor_max_rpm = 715.0 # RC 440
rbt_wheel0_angle = 90.0 
rbt_wheel1_angle = 270.0 
rbt_wheel2_angle = -1.0 
rbt_wheel3_angle = -1.0 
rbt_radius = 0.0375
rbt_wheel_thickness = 0.005
max_wheel_rad_s = (rbt_motor_max_rpm / 60) * 2 * np.pi
max_v = max_wheel_rad_s * rbt_wheel_radius
max_w = np.rad2deg(max_v / (rbt_radius + rbt_wheel_thickness))
max_pos = max(width / 2, (length / 2) + penalty_length)
NORM_BOUNDS = 1.2"""



goal_width = 0.4 
ball_radius = 0.0215 
penalty_width = 0.8 
goal_depth = 0.1 
rbt_kicker_width = -1.0 
penalty_length = 0.15 
length = 2.2 
width = 1.8
rbt_distance_center_kicker = -1.0 
rbt_kicker_thickness = -1.0 
rbt_wheel_radius = 0.02 # RC 0.026
rbt_motor_max_rpm = 570.0 # RC 440
rbt_wheel0_angle = 90.0 
rbt_wheel1_angle = 270.0 
rbt_wheel2_angle = -1.0 
rbt_wheel3_angle = -1.0 
rbt_radius = 0.0375
rbt_wheel_thickness = 0.005
max_wheel_rad_s = (rbt_motor_max_rpm / 60) * 2 * np.pi
max_v = max_wheel_rad_s * rbt_wheel_radius
max_w = np.rad2deg(max_v / (rbt_radius + rbt_wheel_thickness))
max_pos = max(width / 2, (length / 2) + penalty_length)
NORM_BOUNDS = 1.2
