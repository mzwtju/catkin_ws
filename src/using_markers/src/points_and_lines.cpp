#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <iostream>
#include <rviz_visual_tools/rviz_visual_tools.h>
// C++
#include <string>
#include <vector>

double px,py,pz = 0;
geometry_msgs::PoseStamped pose_stamped;
geometry_msgs::Pose pose;
std::vector<geometry_msgs::PoseStamped> path;
EigenSTL::vector_Vector3d path2;
std::vector<geometry_msgs::Pose> waypoints;
void pose_callback(const geometry_msgs::PoseStamped& msg)
{
  pose_stamped = msg;
  px = msg.pose.position.x;
  py = msg.pose.position.y;
  pz = msg.pose.position.z;
  pose = pose_stamped.pose;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map","/rviz_visual_markers"));
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Subscriber pose_sub = n.subscribe("/iris_0/mavros/local_position/pose", 10, pose_callback);
  // ros::Subscriber path_sub = n.subscribe("/iris_0/trajectory", 10, path_callback);
  ros::Rate r(10);
  double test[2][100];
  float f = 0.0;

  while (ros::ok())
  {
    
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;


    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    waypoints.push_back(pose);
    // visual_tools_->publishArrow(pose_stamped,rviz_visual_tools::RED, rviz_visual_tools::LARGE,4,5);
    visual_tools_->publishPath(waypoints, rviz_visual_tools::RED, rviz_visual_tools::LARGE);



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;



    // Points are green
    points.color.b = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;


    
    // // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
      
      geometry_msgs::Point p1,p2;
      p1.x = px;
      p1.y = py;
      p1.z = pz;
      test[0][i] = px;
      test[1][i] = py;
      p2.x = test[0][i];
      p2.y = test[1][i];
      p2.z = pz;
      points.points.push_back(p1);
      line_strip.points.push_back(p2);
      // line_strip.pose.position.x = px;
      // line_strip.pose.position.y = py;
      // line_strip.pose.position.z = pz;
      // The line list needs two points for each line
      // line_list.points.push_back(p);
      // p.z += 1.0;
      // line_list.points.push_back(p);
    }

    // // printf("%f  %f  %f",px,py,pz);
    // line_strip.points.push_back(p);
    // line_strip.pose.position.x = px;
    // line_strip.pose.position.y = py;
    // line_strip.pose.position.z = pz;
     visual_tools_->trigger();

     marker_pub.publish(points);
     marker_pub.publish(line_strip);


    // r.sleep();
    ros::spinOnce();
    f+=0.04;
  }

}
