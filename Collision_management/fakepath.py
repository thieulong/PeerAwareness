from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# number of points
point_num = 100
# make a line
x = list(range(1,point_num))
m = -1
c = 1
y = m*x + c

fakepath = Path()
fakepath.header =
for i in point_num:
    stampedposes = PoseStamped()
    stampedposes.point.x = x[i] 
    stampedposes.point.y = y[i]
    stampedposes.point.z = 0.0
    stampedposes.orientation.x = 0.0
    stampedposes.orientation.y = 0.0
    stampedposes.orientation.z = 0.0
    stampedposes.orientation.w = 1.0
    fakepath.poses.push(stampedposes)