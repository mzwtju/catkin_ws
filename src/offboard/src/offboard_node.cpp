/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
/*我们创建了一个简单的回调函数来储存飞控当前的状态。 这将使得我们可以检查飞机的各项状态，
如是否连接上mavros功能包、是否解锁、当前飞行模式。*/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    /*我们初始化了一个发布者来发布本地的控制指令，还初始化了请求解锁和更改模式的服务。
     请注意，对于您自己的系统，"mavros" 前缀可能不同，因为它将取决于给它的启动文件中的节点指定的名称*/
    ros::Subscriber state_sub = nh.subscribe<uav0/mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<uav0/mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<uav0/mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection 在发布对应消息之前，我们需要等待飞控和MAVROS模块的连接。 在收到心跳包之后，代码便会跳出这个循环。
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    //尽管PX4飞控在NED坐标系下操控飞机，但MAVROS是在ENU系下进行指令传输的。 这也就是为什么我们设置z为+2
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting 在切换到offboard模式之前
    //你必须先发送一些期望点信息到飞控中。 不然飞控会拒绝切换到offboard模式。 在这里, 100 可以被选择为任意数量。
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    //我们现在设置要切换的模式为 OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //剩下的代码都比较容易自学。 我们要先解锁无人机，在切换无人机到offboard模式。 
    //我们每隔五秒去检查一下与飞控的连接等是否一切正常。 在同一个循环中，我们按照指定的频率持续发送期望点信息给飞控。
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

