h_stand = -16
l_body = 15.5
w_body_min = 13.7
w_body_max = 18.6

# default standing robot pose
#                 x    y    z
default_pose = [[ 20, -20, h_stand],
                [  0, -26, h_stand],
                [-20, -20, h_stand],
                [-20,  20, h_stand],
                [  0,  26, h_stand],
                [ 20,  20, h_stand]]

# leg start offset from body
#                     x               y
leg_offset =   [[ (l_body/2), -(w_body_min/2)],   
                [          0, -(w_body_max/2)], 
                [-(l_body/2), -(w_body_min/2)], 
                [-(l_body/2),  (w_body_min/2)],   
                [          0,  (w_body_max/2)], 
                [ (l_body/2),  (w_body_min/2)]]

# servo offset angle
#                  c   f   t
servo_offset = [[-45,  0,  0],
                [  0,  0,  0],
                [ 45,  0,  0],
                [-45,  0,  0],
                [  0,  0,  0],
                [ 45,  0,  0]]

#leg segments dimensions
#                  c   f   t
leg_dim =      [[6.5, 13, 17],
                [6.5, 13, 17],
                [6.5, 13, 17],
                [6.5, 13, 17],
                [6.5, 13, 17],
                [6.5, 13, 17]]