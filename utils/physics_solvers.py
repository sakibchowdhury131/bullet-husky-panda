from .math_solvers import solve_quadratic
g = 9.8
###################################################################################################################################
################### All functions here are written assuming that there will be one single bounce in the table. ####################
###################################################################################################################################

## This function will take the first two positions of the ball and estimate the initial velocity of the ball
## This is necessary because optitrack cannot provide velocity
def estimateInitVelocity(ball_position1, ball_position2, time_gap):
    displacement = (ball_position2[0] - ball_position1[0], ball_position2[1] - ball_position1[1], ball_position2[2] - ball_position1[2])
    velocity = [displacement[i] / time_gap for i in range(len(displacement))]
    return velocity




## This function will provide t1 and t2. 
## t1 = flight time before first bounce
## t2 = flight time after first bounce
def calculate_flight_times(ball_initial_position, robot_initial_position, estimated_initial_velocity, table_height): # this will return the two flight times - t1 and t2
    distance_x, distance_y, distance_z = (robot_initial_position[0] - ball_initial_position[0], robot_initial_position[1] - ball_initial_position[1], robot_initial_position[2] - ball_initial_position[2])

    # height of the ball from the table
    h1 = ball_initial_position[2] - table_height - 0.2 # 0.2m is the height of the red surface


    # now calculate the time for the contact with ground
    a = -0.5*g
    b = estimated_initial_velocity[2]
    c = h1

    t1 = None # it is the time for the ball to hit the ground for the first time

    roots = solve_quadratic(a, b, c)
    for root in roots:
        if root.imag == 0 and root.real > 0: # means complex root, ignore
            t1 = root.real
    ## not calculate how high the projectile can go 
    t2 = distance_x / estimated_initial_velocity[0] - t1 # remaining flight time to reach the robot

    return t1, t2



## This function will provide the height the ball will reach after the bounce. 
## This function assumes there are no energy loss during the collision
def calculate_h2(estimated_initial_velocity, t1, t2): # this will provide the second height that the ball will reach: h2
    ## Velocities after bounce
    vx = estimated_initial_velocity[0]
    vy = estimated_initial_velocity[1]
    vz = estimated_initial_velocity[2] - g*t1
    new_velocity = [vx, vy, -vz] # vz will change sign because now the direction of the projectile in z axis is reversed
    h2 = new_velocity[2]*t2 - 0.5*g*(t2**2)

    return h2


def had_double_bounce(h2):
    if h2 <= 0:
        return True
    else:
        return False

def had_no_bounce(vx, t1, distance_ball_robot):
    if vx*t1 >= distance_ball_robot:
        return True
    else:
        return False


## This function will estimate where the ball will hit the Y-Z plane after the first bounce
def estimate_hitting_point(ball_initial_position, robot_initial_position, estimated_initial_velocity, table_height, base_surface_thickness = 0.2): # this will return the estimated hitting point in the Y-Z plane
    t1, t2 = calculate_flight_times(ball_initial_position, robot_initial_position, estimated_initial_velocity, table_height)
    h2 = calculate_h2(estimated_initial_velocity, t1, t2)

    y = ball_initial_position[1] + estimated_initial_velocity[1] * (t1 + t2)
    z = h2 + table_height + base_surface_thickness
    return (y, z), t1, t2



# will calculate the delta for Y-Z plane
def calculate_delta(real_hitting_point, estimated_hitting_point):
    delta = (real_hitting_point[0] - estimated_hitting_point[0], real_hitting_point[1] - estimated_hitting_point[1])
    return delta
