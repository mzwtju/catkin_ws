#!/usr/bin/env python
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget,AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist,Vector3,Quaternion
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String,Header,Int16
from nav_msgs.msg import Path
import time
# from pyquaternion import Quaternion  ATTENTION
import math
r = 5
w = 0.2
class Leader:
    def __init__(self):

        self.local_pose = PoseStamped()
        self.current_state = State()
        self.current_heading = None
        self.takeoff_height = None
        self.rpy_pose = None
        self.setpoint_pose = PoseStamped()
        self.setpoint_vel=TwistStamped()
        self.setpoint_att = AttitudeTarget()
        self.setpoint_motion = PositionTarget()
        self.global_setpoint = None
        self.path = Path()
        self.global_position = NavSatFix()
        self.cnt = Int16()

        self.arm_state = False
        self.offboard_state = False
        self.flag = 0
        self.flight_mode = None
        self.state = Int16()
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.global_pose_sub = rospy.Subscriber("/iris_0/mavros/global_position/global", NavSatFix, self.global_pose_callback)
        self.mavros_sub = rospy.Subscriber("/iris_0/mavros/state", State, self.mavros_state_callback)
        self.uav1_state_sub =rospy.Subscriber('/iris_1/state', Int16, self.uav1_state_callback)
        self.uav2_state_sub =rospy.Subscriber('/iris_2/state', Int16, self.uav2_state_callback)
        '''
        ros publishers
        '''
        # self.pose_setpoint_pub = rospy.Publisher('/iris_0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # self.vel_setpoint_pub = rospy.Publisher('/iris_0/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        # self.att_setpoint_pub = rospy.Publisher('/iris_0/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.motion_setpoint_pub = rospy.Publisher('/iris_0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.trajectory_pub = rospy.Publisher('/iris_0/trajectory', Path, queue_size=10)
        self.timecnt_pub = rospy.Publisher('/iris_0/timecnt',Int16, queue_size=10)
        self.state_pub = rospy.Publisher('/iris_0/state', Int16, queue_size=10)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/iris_0/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/iris_0/mavros/set_mode', SetMode)
    
    def start(self):
        rospy.init_node('leader_control', anonymous=True)
        print("uav0 terminal")
        now = None
        count = 0
        all_ready = 0
        self.uav1_state = Int16()
        self.uav2_state = Int16()
        # self.get_setpoint_pose()
        # self.get_setpoint_vel(0,0,0.5)
        # self.get_setpoint_att()
        self.get_setpoint_motion(0,0,0,0,0,0.1,0,0,0,0)
        rate = rospy.Rate(20) # 20hz 0.05s
        for i in range(100):
            self.motion_setpoint_pub.publish(self.setpoint_motion)
            rate.sleep()
        self.flight_mode = "OFFBOARD"
        #while not rospy.is_shutdown() and self.current_state.connected:
        while not rospy.is_shutdown():
            time = rospy.get_rostime() 
            self.trajectory_pub.publish(self.path)
            count=count+1
            if self.current_state.mode!="OFFBOARD" and count == 20:
                self.flight_mode_switch()
            if self.current_state.armed == False and self.current_state.mode == "OFFBOARD" and count == 20:
                self.arm()
            # self.pose_setpoint_pub.publish(self.setpoint_pose)
            # self.att_setpoint_pub.publish(self.setpoint_att)
            # self.vel_setpoint_pub.publish(self.setpoint_vel)
            if self.local_pose.pose.position.z >=5.4 and self.local_pose.pose.position.z <= 6:
                    self.state.data = 1
                    if all_ready == 1:
                        vx = -w*r*math.sin(w*self.cnt.data*0.05)
                        vy = -w*r*math.cos(w*self.cnt.data*0.05) # w r wrong elements
                        self.cnt.data = self.cnt.data + 1
                        z = 0
            elif self.local_pose.pose.position.z >6:
                vx = w*r*math.sin(w*self.cnt.data*0.05)
                vy = w*r*math.cos(w*self.cnt.data*0.05) 
                self.cnt.data += 1
                z = 0
            elif self.local_pose.pose.position.z <5:
                z = 0.5
                vx = 0
                vy = 0
            if self.state_pub.impl is not None:
                self.state_pub.publish(self.state)
            if self.uav1_state.data == 1 and self.uav2_state.data == 1 and self.state.data == 1:
                all_ready = 1
                self.uav1_state_sub.unregister()
                self.uav2_state_sub.unregister()
                self.state_pub.unregister()

            T = 2*math.pi/w
            if self.cnt.data*0.05>T :
                self.cnt.data = 0
            self.timecnt_pub.publish(self.cnt)
            #     vy = 0
            # print(time.secs)
            self.get_setpoint_motion(0,0,5.6,vx,vy,0,0,0,0,0,0)
            self.motion_setpoint_pub.publish(self.setpoint_motion)
            if count == 20:
                now = rospy.get_rostime()
                print("local_position\tx:%.2f y:%.2f z:%.2f"%(self.local_pose.pose.position.x,\
                                        self.local_pose.pose.position.y,\
                                        self.local_pose.pose.position.z))
                # print("local_attitude\t roll:%.2f pitch:%.2f yaw:%.2f\r"%(self.rpy_pose[2],\
                #                                                          self.rpy_pose[1],\
                #                                                          self.rpy_pose[0]))
                print("velocity_vx %.3f velocity_y %.3f"%(vx,vy))                                                
                count = 0
                print(self.state.data,self.uav1_state.data,self.uav2_state.data,all_ready)
            # print("time: %.8f"%(time.nsecs-now.nsecs)*1e-9)
            # print("time: %.8f"%(time.secs+time.nsecs*1e-9))

            rate.sleep()
    def uav1_state_callback(self,msg):
        self.uav1_state = msg
    def uav2_state_callback(self,msg):
        self.uav2_state = msg

    def local_pose_callback(self,msg):
        self.local_pose = msg
        self.path.poses.append(msg) 
        self.path.header = msg.header
        self.rpy_pose = tf.transformations.euler_from_quaternion([self.local_pose.pose.orientation.w,self.local_pose.pose.orientation.x,self.local_pose.pose.orientation.y,self.local_pose.pose.orientation.z])
    def global_pose_callback(self,msg):
        self.global_position = msg
    def mavros_state_callback(self,msg):
        self.current_state = msg

    def get_setpoint_pose(self):
        self.setpoint_pose.header = Header()
        self.setpoint_pose.pose.position.x = 0
        self.setpoint_pose.pose.position.y = 0
        self.setpoint_pose.pose.position.z = 2

    def get_setpoint_vel(self,x,y,z):
        self.setpoint_vel.header = Header()
        self.setpoint_vel.twist.linear.x = x
        self.setpoint_vel.twist.linear.y = y
        self.setpoint_vel.twist.linear.z = z
    
    def get_setpoint_att(self):
        self.setpoint_att.body_rate = Vector3()
        self.setpoint_att.header = Header()
        self.setpoint_att.header.frame_id = "base_footprint"
        self.setpoint_att.orientation = Quaternion(*quaternion_from_euler(0, 0,
                                                                 0))
        self.setpoint_att.thrust = 0.6
        self.setpoint_att.type_mask = 7  # ignore body rate

    def get_setpoint_motion(self,x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        self.setpoint_motion.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_motion.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY  \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE
        self.setpoint_motion.position.x = x
        self.setpoint_motion.position.y = y
        self.setpoint_motion.position.z = z

        self.setpoint_motion.velocity.x = vx
        self.setpoint_motion.velocity.y = vy
        self.setpoint_motion.velocity.z = vz
        
        self.setpoint_motion.acceleration_or_force.x = afx
        self.setpoint_motion.acceleration_or_force.y = afy
        self.setpoint_motion.acceleration_or_force.z = afz

        self.setpoint_motion.yaw = yaw
        self.setpoint_motion.yaw_rate = yaw_rate
    def arm(self):
        if self.armService(True):
            print("Armed")
            return True
        else:
            print("Vehicle Arming Failed!")
            return False
    
    def flight_mode_switch(self):
        if self.flightModeService(custom_mode=self.flight_mode):
            print("switch to "+self.flight_mode)
            return True
        else:
            print(self.flight_mode+"Failed")
            return False


if __name__ == '__main__':
    leader = Leader()
    leader.start()

