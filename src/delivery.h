#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/console.h>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std ;

class delivery
{
  public:
    delivery(ros::NodeHandle) ;
    ~delivery() {}
    
  private:
    ros::Subscriber subResult ;
    ros::Publisher pubWaypoint ;
    move_base_msgs::MoveBaseActionResult result ;
    geometry_msgs::Twist waypoint ;
    int currentWP ;
    int totalWP ;
    vector< vector<double> > allWaypoints ;
    
    void waypointCallback(const move_base_msgs::MoveBaseActionResult&) ;
};

delivery::delivery(ros::NodeHandle nh){
  subResult = nh.subscribe("move_base/result", 10, &delivery::waypointCallback, this) ;
  pubWaypoint = nh.advertise<geometry_msgs::Twist>("cmd_map_goal", 10) ;
  currentWP = 1 ;
  char buffer[50] ;
  ros::param::get("num_wps", totalWP) ;
  for (int i = 0; i < totalWP; i++){
    vector<double> tempWP ;
    double temp ;
    sprintf(buffer,"v%d/x",i) ;
    ros::param::get(buffer, temp) ;
    tempWP.push_back(temp) ;
    sprintf(buffer,"v%d/y",i) ;
    ros::param::get(buffer, temp) ;
    tempWP.push_back(temp) ;
    allWaypoints.push_back(tempWP) ;
  }
}

void delivery::waypointCallback(const move_base_msgs::MoveBaseActionResult& msg){
  result = msg ;
  if (msg.status.status == 6 || msg.status.status == 2 || msg.status.status == 4 || msg.status.status == 5){ // waypoint was preempted or aborted
    ROS_INFO("Unable to reach waypoint! Selecting new waypoint...") ;
  }
  
  int newWP = currentWP ;

  while (newWP == currentWP)
    newWP = rand() % totalWP ;

  currentWP = newWP ;
  
  waypoint.linear.x = allWaypoints[currentWP][0] ;
  waypoint.linear.y = allWaypoints[currentWP][1] ;
  waypoint.linear.z = 0 ;
  waypoint.angular.x = 0 ;
  waypoint.angular.y = 0 ;
  waypoint.angular.z = 0 ;
  //ROS_INFO_STREAM("Sending new waypoint ("<< waypoint.linear.x << "," << waypoint.linear.y << ")") ;
  pubWaypoint.publish(waypoint) ;
}
