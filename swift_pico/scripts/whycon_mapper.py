def pixel_to_whycon(imgx, imgy):
    goal_x= 0.02537*imgx - 12.66
    goal_y= 0.02534*imgy - 12.57
    goal_z= 27.0
    goal = [goal_x, goal_y, goal_z]
    return goal
