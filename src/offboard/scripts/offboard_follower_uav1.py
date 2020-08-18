#!/usr/bin/env python
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget,AttitudeTarget,RCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist,Vector3,Quaternion
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String,Header
import time
import math
import tty, termios,sys, select, os
# 
vel_max_xy = 0.5
vel_max_z = 0.5
KP_x = -0.4
KP_y = -0.4
KP_z = 0.5
formation = [[5,5,0],[5,0,0],[0,5,0]]


class Follower:
    def __init__(self):

        self.local_pose = PoseStamped()
        self.global_pose = NavSatFix()
        self.leader_pose = PoseStamped()
        self.leader_global_pose = NavSatFix()
        self.current_state = State()
        self.relative_pose = PoseStamped()
        self.rcin = RCIn()
        self.current_heading = None
        self.takeoff_height = None
        self.rpy_pose = None
        self.count = 0

        self.setpoint_pose = PoseStamped()
        self.setpoint_vel = TwistStamped()
        self.setpoint_att = AttitudeTarget()
        self.setpoint_motion = PositionTarget()
        self.global_setpoint = None
        # self.formation_config 
        
        self.dist = 0
        self.bearing = 0

        self.arm_state = False
        self.offboard_state = False
        self.flag = 0
        self.flight_mode = None
        self.followid = 1
        self.formation_id = 1
        '''
        ros subscribers
        '''
        self.rcin_sub = rospy.Subscriber('/uav0/mavros/rc/in', RCIn, self.rcin_callback)
        self.leader_local_pose_sub = rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.leader_pose_callback)
        self.leader_global_pose_sub = rospy.Subscriber('/uav0/mavros/global_position/global', NavSatFix, self.leader_global_pose_callback)
        self.local_pose_sub = rospy.Subscriber('/uav'+str(self.followid)+'/mavros/local_position/pose', PoseStamped, self.local_pose_callback)
        self.global_pose_sub = rospy.Subscriber('/uav'+str(self.followid)+'/mavros/global_position/global', NavSatFix, self.global_pose_callback)
        self.mavros_sub = rospy.Subscriber('/uav'+str(self.followid)+'/mavros/state', State, self.mavros_state_callback)
        '''
        ros publishers
        '''
        self.pose_setpoint_pub = rospy.Publisher('/uav'+str(self.followid)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.vel_setpoint_pub = rospy.Publisher('/uav'+str(self.followid)+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.att_setpoint_pub = rospy.Publisher('/uav'+str(self.followid)+'/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.motion_setpoint_pub = rospy.Publisher('/uav'+str(self.followid)+'/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/uav'+str(self.followid)+'/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/uav'+str(self.followid)+'/mavros/set_mode', SetMode)
    
    def start(self):
        rospy.init_node('follower_'+str(self.followid)+'_control', anonymous=True)
        print("follower "+str(self.followid)+" terminal")
        number = 1
        self.get_setpoint_pose()
        self.get_setpoint_vel(0,0,0.5)
        self.get_setpoint_att()
        self.get_setpoint_motion(0,0,0,0,0,0.5,0,0,0,0,0)
        rate = rospy.Rate(20) # 20hz
        for i in range(100):
            self.motion_setpoint_pub.publish(self.setpoint_motion)
            rate.sleep()
        self.flight_mode = "OFFBOARD"
        #while not rospy.is_shutdown() and self.current_state.connected:
        while not rospy.is_shutdown():
            key = getKey()
            self.count = self.count +1
            # if self.current_state.mode!="OFFBOARD" and self.count == 19 :
            #     self.flight_mode_switch()
            # if self.current_state.armed == False and self.current_state.mode == "OFFBOARD" and self.count == 19:
            #     self.arm()
            if(self.formation_id == 1):
                number = 1
                print("switch to formation 1")
            elif(self.formation_id == 2):
                number = 2
                print("switch to formation 2")
            elif(self.formation_id ==3):
                number = 3
                print("switch to formation 3")

            formation_p_x = formation[int(number)-1][0]
            formation_p_y = formation[int(number)-1][1]
            formation_p_z = formation[int(number)-1][2]

            formation_vel_x = delta_vel(formation_p_x,self.relative_pose.pose.position.x,KP_x,vel_max_xy) #target - current 
            formation_vel_y = delta_vel(formation_p_y,self.relative_pose.pose.position.y,KP_y,vel_max_xy)
            formation_vel_z = delta_vel(formation_p_z,self.relative_pose.pose.position.z,KP_z,vel_max_z)
            
            if self.dist<3:
                avoid_vel_z = (3-self.relative_pose.pose.position.z)*KP_z
                if avoid_vel_z>1:
                    avoid_vel_z = 1
            elif self.dist>3:
                avoid_vel_z = 0    
            
            self.get_setpoint_motion(0,0,0,formation_vel_x,formation_vel_y,formation_vel_z+avoid_vel_z,0,0,0,0,0)

            self.count=self.count+1
            # self.pose_setpoint_pub.publish(self.setpoint_pose)
            # self.att_setpoint_pub.publish(self.setpoint_att)
            self.motion_setpoint_pub.publish(self.setpoint_motion)
            # self.motion_setpoint_pub.publish(self.setpoint_motion)

            # if self.local_pose.pose.position.z >2:
            #     self.get_setpoint_motion(0,0,0,1,0,0,0,0,0,0,0)
            # else:
            #     self.get_setpoint_motion(0,0,0,1,0,0.5,0,0,0,0,0)
            if self.count == 20:
                # print("local_position\t x:%.2f y:%.2f z:%.2f"%(self.local_pose.pose.position.x,\
                #                         self.local_pose.pose.position.y,\
                #                         self.local_pose.pose.position.z))
                # print("local_attitude\t roll:%.2f pitch:%.2f yaw:%.2f"%(self.rpy_pose[2],\
                #                                                          self.rpy_pose[1],\
                #                                                          self.rpy_pose[0]))
                print("dis to leader%.2f bearing to leader%.2f"%(self.dist,math.degrees(self.bearing)) )
                # print("relative_local_position\t x:%.2f y:%.2f z:%.2f"%(self.relative_pose.pose.position.x,\
                #                         self.relative_pose.pose.position.y,\
                #                         self.relative_pose.pose.position.z))
                print("formation number:%d"%number)
                # print("leader_position\t x:%.7f y:%.7f "%(self.leader_global_pose.latitude,\
                #                         self.leader_global_pose.longitude))
                # print("position\t x:%.7f y:%.7f "%(self.global_pose.latitude,\
                #                         self.global_pose.longitude))
                print("vel_x:%.2f vel_y:%.2f vel_z:%.2f"%(formation_vel_x,\
                        formation_vel_y,\
                        formation_vel_z+avoid_vel_z))
                self.count = 0
            rate.sleep()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    def rcin_callback(self,msg):
        self.rcin = msg
        if self.rcin.channels[6]<1100:
            self.formation_id = 1
        elif self.rcin.channels[6] >1200 and self.rcin.channels[6]<1600:
            self.formation_id = 2
        elif self.rcin.channels[6]>1600:
            self.formation_id = 3

    def leader_pose_callback(self,msg):
        self.leader_pose = msg

    def leader_global_pose_callback(self,msg):
        self.leader_global_pose = msg
        
    def global_pose_callback(self,msg):
        self.global_pose = msg
        self.dist,self.bearing = haversine([self.global_pose.latitude,self.global_pose.longitude],\
                                            [self.leader_global_pose.latitude,self.leader_global_pose.longitude])
    def local_pose_callback(self,msg):
        self.local_pose = msg
        self.calculate_relative_pose()
        self.rpy_pose = tf.transformations.euler_from_quaternion([self.local_pose.pose.orientation.w,self.local_pose.pose.orientation.x,self.local_pose.pose.orientation.y,self.local_pose.pose.orientation.z])

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
        self.setpoint_motion.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
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

    def calculate_relative_pose(self):
        # self.relative_pose.pose.position.x = self.local_pose.pose.position.x - self.leader_pose.pose.position.x
        # self.relative_pose.pose.position.y = self.local_pose.pose.position.y - self.leader_pose.pose.position.y
        self.relative_pose.pose.position.x = self.dist*math.cos(self.bearing)
        self.relative_pose.pose.position.y = self.dist*math.sin(self.bearing)
        self.relative_pose.pose.position.z = self.local_pose.pose.position.z - self.leader_pose.pose.position.z

def haversine(coord1, coord2):
    R = 6372800  # Earth radius in meters
    #bearing = from coord1 to coord2 heading
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    
    phi1, phi2 = math.radians(lat1), math.radians(lat2) 
    dphi       = math.radians(lat2 - lat1)
    dlambda    = math.radians(lon2 - lon1)
    
    a = math.sin(dphi/2)**2 + \
        math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    
    bearing = math.atan2(math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1), math.sin(lon2-lon1)*math.cos(lat2))
    
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1 - a)),bearing

def delta_vel(target_pos, current_pos, KP, vel_max):
    delta_vel = KP*(target_pos-current_pos)
    if delta_vel > vel_max:
        delta_vel = vel_max
    elif delta_vel<-vel_max:
        delta_vel = -vel_max 
    return delta_vel

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    follower = Follower()
    follower.start()

