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
        self.cnt.data = 0
        self.arm_state = False
        self.offboard_state = False
        self.flag = 0
        self.type = 448
        self.flight_mode = None
        self.state = Int16()
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.global_pose_sub = rospy.Subscriber("/uav0/mavros/global_position/global", NavSatFix, self.global_pose_callback)
        self.mavros_sub = rospy.Subscriber("/uav0/mavros/state", State, self.mavros_state_callback)
        self.uav1_state_sub =rospy.Subscriber('/uav1/state', Int16, self.uav1_state_callback)
        self.uav2_state_sub =rospy.Subscriber('/uav2/state', Int16, self.uav2_state_callback)
        '''
        ros publishers
        '''
        self.motion_setpoint_pub = rospy.Publisher('/uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.trajectory_pub = rospy.Publisher('/uav0/trajectory', Path, queue_size=10)
        self.state_pub = rospy.Publisher('/uav0/state', Int16, queue_size=10)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)
    
    def start(self):
        rospy.init_node('leader_control', anonymous=True)
        print("uav0 terminal")
        now = None
        count = 0
        all_ready = 0
        dx = 0
        dy = 0
        x = 0
        y = 0
        #type = 448 #ignore AxAyAz
        T = 2*math.pi/w
        self.uav1_state = Int16()
        self.uav2_state = Int16()
        # self.get_setpoint_pose()
        # self.get_setpoint_vel(0,0,0.5)
        # self.get_setpoint_att()
        self.get_setpoint_motion(self.type,0,0,0,0,0,0.1,0,0,0,0)
        rate = rospy.Rate(20) # 20hz 0.05s
        for i in range(100):
            self.motion_setpoint_pub.publish(self.setpoint_motion)
            rate.sleep()
        self.flight_mode = "OFFBOARD"
        #while not rospy.is_shutdown() and self.current_state.connected:
        while not rospy.is_shutdown():
            count=count+1
            self.trajectory_pub.publish(self.path)
            if self.current_state.mode!="OFFBOARD" and count == 20:
                self.flight_mode_switch()
            if self.current_state.armed == False and self.current_state.mode == "OFFBOARD" and count == 20:
                self.arm()
            # self.pose_setpoint_pub.publish(self.setpoint_pose)
            # self.att_setpoint_pub.publish(self.setpoint_att)
            # self.vel_setpoint_pub.publish(self.setpoint_vel)
            if self.local_pose.pose.position.z >=1.7 and self.local_pose.pose.position.z <= 2.5:
                    self.state.data = 1
                    if all_ready == 1:
                        self.type = 451
                        x = 0
                        y = 0
                        dx = cx -r + r*math.cos(w*self.cnt.data*0.05)
                        dy = cy - r*math.sin(w*self.cnt.data*0.05)
                        vx = -(self.local_pose.pose.position.x -dx)
                        vy = -(self.local_pose.pose.position.y -dy)
                        # vx = -w*r*math.sin(w*self.cnt.data*0.05)
                        # vy = -w*r*math.cos(w*self.cnt.data*0.05) # w r wrong elements
                        self.cnt.data = self.cnt.data + 1
                        z = 0
                    elif all_ready == 0:
                        x = 0
                        y = 0
                        vx = 0
                        vy = 0
            elif self.local_pose.pose.position.z >2.5:
                vx = w*r*math.sin(w*self.cnt.data*0.05)
                vy = w*r*math.cos(w*self.cnt.data*0.05) 
                self.cnt.data += 1
                z = 0
            elif self.local_pose.pose.position.z <1.7:
                x = 0
                y = 0
                vx = 0
                vy = 0
            if self.state_pub.impl is not None:
                self.state_pub.publish(self.state)
            if self.uav1_state.data == 1 and self.uav2_state.data == 1 and self.state.data == 1 and all_ready == 0:
            # if self.state.data == 1 and all_ready == 0:
                all_ready = 1
                cx = self.local_pose.pose.position.x
                cy = self.local_pose.pose.position.y
                self.uav1_state_sub.unregister()
                self.uav2_state_sub.unregister()
                self.state_pub.unregister()
                print("circular center point \tx:%.2f y:%.2f"%(cx,cy))
            if self.cnt.data*0.05>T :
                self.cnt.data = 0
            vx = vel_constrain(vx,1.2)
            vy = vel_constrain(vy,1.2)
            self.get_setpoint_motion(self.type,x,y,2,vx,vy,0,0,0,0,0,0)
            self.motion_setpoint_pub.publish(self.setpoint_motion)
            if count == 20:
                # now = rospy.get_rostime()
                # print("local_position x:%.2f y:%.2f z:%.2f"%(self.local_pose.pose.position.x,\
                #                         self.local_pose.pose.position.y,\
                #                         self.local_pose.pose.position.z))
                # print("circular orbit x:%.2f y:%.2f"%(dx,dy))
                # print("local_attitude\t roll:%.2f pitch:%.2f yaw:%.2f\r"%(self.rpy_pose[2],\
                #                                                          self.rpy_pose[1],\
                #                                                          self.rpy_pose[0]))
                # print("velocity_vx %.3f velocity_y %.3f"%(vx,vy))   
                # print("\r")                                             
                count = 0
                # print(self.state.data,self.uav1_state.data,self.uav2_state.data,all_ready)
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


    def get_setpoint_motion(self,type = 0,x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        self.setpoint_motion.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_motion.type_mask = type
                            
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
def vel_constrain(target,vel_max):
    if target > vel_max:
        target = vel_max
    elif target<-vel_max:
        target = -vel_max 
    return target

if __name__ == '__main__':
    leader = Leader()
    leader.start()

