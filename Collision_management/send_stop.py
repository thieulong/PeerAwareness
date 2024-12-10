# if robot1 is about to reach stopping_point1
if abs(robot.x - stopping_point1.x) < 0.05 and abs(robot.y - stopping_point1.y) < 0.05:
    # if robot1 receives stop signal before it reach stopping_point1, then stop
    if (stop == 1):
        # stop
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
    else:        
        # send stop signal to other robots since robot1 has occupied the 'intersection'
        publish("occupied")
        # resume or keep moving
        cmd_vel.linear.x = vel_x
        cmd_vel.angular.z = vel_z
# if robot1 pass going_point1, send go signal
if (robot.x - stopping_point1.x) > 0.05 and (robot.y - stopping_point1.y) > 0.05:
    publish("vacant")

