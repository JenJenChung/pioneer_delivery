#include <ros/ros.h>
#include "delivery.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "delivery_waypoint") ;

  ros::NodeHandle nHandle ;
  
  delivery deliveryWP(nHandle) ;
  
  ros::spin();
  return 0;
}
