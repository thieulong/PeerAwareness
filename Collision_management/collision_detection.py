# path1 is the path planned by robot 1
path1: [
    pose11,
    pose12,
    ...,
    pose1n, 
]

# path2 is the path planned by robot 2
path2: [
    pose21,
    pose22,
    ...,
    pose2n, 
]

path1_filtered # filtered version of path1
path2_filtered # filtered version of path2

stopping_point1 = None # stopping point for robot 1
going_point1 = None # going point for robot 1
index1a = None # index the first waypoints closest to the collision point for robot 1
index1b = None # index the second waypoints closest to the collision point for robot 1

stopping_point2 = None # stopping point for robot 2
going_point2 = None # going point for robot 2
index2a = None # index the first waypoints closest to the collision point for robot 2
index2b = None # index the second waypoints closest to the collision point for robot 2


index = 0 # index to loop through all the poses on each path

while (path1 has not reached the end) or (path2 has not reached the end):
    # collision point detected
    if abs(path1[index].postion.x - path2[index].position.x) < 0.02 and abs(path1[index].postion.y - path2[index].position.y) < 0.02:
        
        # look for the two filtered waypoints that are closest to the collision point
        d_smallest_1 = math.inf
        d_smallest_2 = math.inf
        for (i < path1_filtered.length):
            d = math.sqrt((path1[index].position.x - path1_filtered[i].position.x)**2  + (path1[index].position.y - path1_filtered[i].position.y)**2)
            if d <= d_smallest_1:
                d_smallest_2 = d_smallest_1
                d_smallest_1 = d
                index1b = index1a
                index1a = i
            elif d < d_smallest_2: 
                d_smallest_2 = d
                index1b = i

        # check which waypoint is closer to the robot
        d1 = math.sqrt((path1_filtered[index1a].position.x - robot.x)**2 + (path1_filtered[index1a].position.y - robot.y)**2)
        d2 = math.sqrt((path1_filtered[index1b].position.x - robot.x)**2 + (path1_filtered[index1b].position.y - robot.y)**2)
        if d1 < d2: 
            stopping_point1 = path1_filtered[index1a]
            going_point1 = path1_filtered[index1b]
        else:
            stopping_point1 = path1_filtered[index1b]
            going_point1 = path1_filtered[index1a]

        # repeat all steps above, in the while loop, to path2
        d_smallest_1 = math.inf
        d_smallest_2 = math.inf
        for (j < path2_filtered.length):
            d = math.sqrt((path2[index].position.x - path2_filtered[i].position.x)**2  + (path2[index].position.y - path2_filtered[i].position.y)**2)
            if d < d_smallest_1:
                d_smallest_2 = d_smallest_1
                d_smallest_1 = d
                index2b = index2a
                index2a = i
            elif d < d_smallest_2: 
                d_smallest_2 = d
                index1b = i

        d1 = math.sqrt((path2_filtered[index1a].position.x - robot.x)**2 + (path2_filtered[index1a].position.y - robot.y)**2)
        d2 = math.sqrt((path2_filtered[index1b].position.x - robot.x)**2 + (path2_filtered[index1b].position.y - robot.y)**2)
        if d1 < d2: 
            stopping_point1 = path2_filtered[index1a]
            going_point1 = path2_filtered[index1b]
        else:
            stopping_point1 = path2_filtered[index1b]
            going_point1 = path2_filtered[index1a]
    index++ 
