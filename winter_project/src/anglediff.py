def anglewrapper(unwrapped_theta, theta_final):

    #checking all 4 quadrants for each case with goal and current orientation in different quadrants

    if theta_final >= 0 and theta_final <= pi/2:

        if unwrapped_theta >= 0 and unwrapped_theta <= pi/2:
            if unwrapped_theta > theta_final:
                angular_error = -abs(unwrapped_theta-theta_final)
            else:
                angular_error = abs(theta_final-unwrapped_theta)

        if unwrapped_theta > pi/2 and unwrapped_theta <= pi:
            angular_error = -abs(unwrapped_theta-theta_final)

        if unwrapped_theta > pi and unwrapped_theta <= 3*pi/2:
            if abs(theta_final-unwrapped_theta) < (theta_final+(2*pi-unwrapped_theta)):
                angular_error = -abs(theta_final-unwrapped_theta)
            else:
                angular_error = abs((theta_final+(2*pi-unwrapped_theta)))
        else:
            angular_error = abs((theta_final+(2*pi-unwrapped_theta)))

    if theta_final > pi/2 and theta_final <= pi:

        if unwrapped_theta >= 0 and unwrapped_theta <= pi/2:
            angular_error = abs(theta_final-unwrapped_theta)

        if unwrapped_theta > pi/2 and unwrapped_theta <= pi:
            if unwrapped_theta > theta_final:
                angular_error = -abs(unwrapped_theta-theta_final)
            else:
                angular_error = abs(theta_final-unwrapped_theta)

        if unwrapped_theta > pi and unwrapped_theta <= 3*pi/2:
            angular_error = -abs(theta_final-unwrapped_theta)

        else:
            if abs(theta_final-unwrapped_theta) < (theta_final+(2*pi-unwrapped_theta)):
                angular_error = -abs(theta_final-unwrapped_theta)
            else:
                angular_error = abs((theta_final+(2*pi-unwrapped_theta)))

    if theta_final > pi and theta_final <= 3*pi/2:

        if unwrapped_theta >= 0 and unwrapped_theta <= pi/2:
            if abs(theta_final-unwrapped_theta) < abs((2*pi-theta_final)+(unwrapped_theta)):
                angular_error = abs(theta_final-unwrapped_theta)
            else:
                angular_error = -abs((2*pi-theta_final)+(unwrapped_theta))

        if unwrapped_theta > pi/2 and unwrapped_theta <= pi:
            angular_error = abs(theta_final-unwrapped_theta)

        if unwrapped_theta > pi and unwrapped_theta <= 3*pi/2:
            if unwrapped_theta > theta_final:
                angular_error = -abs(unwrapped_theta-theta_final)
            else:
                angular_error = abs(theta_final-unwrapped_theta)

        else:
            angular_error = -abs(theta_final-unwrapped_theta)

    if theta_final > 3*pi/2 and theta_final <= 2*pi:

        if unwrapped_theta >= 0 and unwrapped_theta <= pi/2:
            angular_error = -abs((2*pi-theta_final)+unwrapped_theta)

        if unwrapped_theta > pi/2 and unwrapped_theta <= pi:
            if abs(theta_final-unwrapped_theta) < (theta_final+(2*pi-unwrapped_theta)):
                angular_error = abs(theta_final-unwrapped_theta)
            else:
                angular_error = -abs(((2*pi-theta_final)+(unwrapped_theta)))

        if unwrapped_theta > pi and unwrapped_theta <= 3*pi/2:
            angular_error = abs(theta_final-unwrapped_theta)

        else:
            if unwrapped_theta > theta_final:
                angular_error = -abs(unwrapped_theta-theta_final)
            else:
                angular_error = abs(theta_final-unwrapped_theta)

    return angular_error, rotation_direction

#####################################################
    if abs(theta_current-theta_final) >= pi:
        angular_error = 2*pi-abs(theta_current-theta_final)
    else:
        angular_error = abs(theta_current-theta_final)

    if theta_current > theta_final:
        rotation_direction = -1
    else:
        rotation_direction = 1

    return angular_error, rotation_direction
