import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String 
import time
import math
import numpy 
import sys

uav_num = int(sys.argv[1])
pose = [None]*uav_num
pose_sub = [None]*uav_num
avoid_vel_pub = [None]*uav_num
avoid_kp = 1
avoid_radius = 1
aid_vec1 = [1, 0, 0]
aid_vec2 = [0, 1, 0]
uavs_avoid_vel = [Vector3()] * uav_num

def pose_callback(msg, id):
    pose[id] = msg

rospy.init_node('avoid')
for i in range(uav_num):
    pose_sub[i] = rospy.Subscriber('/uav'+str(i+1)+'/mavros/local_position/pose',PoseStamped,pose_callback,i)
    avoid_vel_pub[i] = rospy.Publisher("/xtdrone/uav"+str(i+1)+"/avoid_vel", Vector3,queue_size=10)

time.sleep(1)
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    for i in range(uav_num):
        position1 = numpy.array([pose[i].pose.position.x, pose[i].pose.position.y, pose[i].pose.position.z])
        for j in range(1, uav_num-i):
            position2 = numpy.array([pose[i+j].pose.position.x, pose[i+j].pose.position.y, pose[i+j].pose.position.z])
            dir_vec = position1-position2
            if numpy.linalg.norm(dir_vec) < avoid_radius:
                cos1 = dir_vec.dot(aid_vec1)/(numpy.linalg.norm(dir_vec) * numpy.linalg.norm(aid_vec1))
                cos2 = dir_vec.dot(aid_vec2)/(numpy.linalg.norm(dir_vec) * numpy.linalg.norm(aid_vec2))
                if  abs(cos1) < abs(cos2):
                    avoid_vel = avoid_kp * numpy.cross(dir_vec, aid_vec1)/numpy.linalg.norm(numpy.cross(dir_vec, aid_vec1))
                else:
                    avoid_vel = avoid_kp * numpy.cross(dir_vec, aid_vec2)/numpy.linalg.norm(numpy.cross(dir_vec, aid_vec2))
                uavs_avoid_vel[i] = Vector3(uavs_avoid_vel[i].x+avoid_vel[0],uavs_avoid_vel[i].y+avoid_vel[1],uavs_avoid_vel[i].z+avoid_vel[2])
                uavs_avoid_vel[i+j] = Vector3(uavs_avoid_vel[i+j].x-avoid_vel[0],uavs_avoid_vel[i+j].y-avoid_vel[1],uavs_avoid_vel[i+j].z-avoid_vel[2])
    for i in range(uav_num):
        avoid_vel_pub[i].publish(uavs_avoid_vel[i])
    uavs_avoid_vel = [Vector3()] * uav_num
    rate.sleep()
            
            
