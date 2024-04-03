#include"ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "controller/final_poseActionAction.h"
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_msgs/TFMessage.h>
//#include <tf/TFMessage.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/PoseWithCovariance.h>
#include<nav_msgs/OccupancyGrid.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
void msgCallback(const tf2_msgs::TFMessagePtr& tfp){
    for(int i= 0;i<1;i++)
        ROS_INFO("TRANSLATION IS OF [%f,%f,%f]",tfp->transforms[i].transform.translation.x,tfp->transforms[i].transform.translation.y,tfp->transforms[i].transform.translation.z);
}

void msgCallback2(const nav_msgs::OccupancyGridPtr& tfp){
    ROS_INFO("TRANSLATION IS OF [%f,%f,%f]",tfp->info.origin.position.x,tfp->info.origin.position.y,tfp->info.origin.position.z);
}

void msgCallback3(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& tfp){
    ROS_INFO("TRANSLATION IS OF [%f,%f,%f]",tfp->pose.pose.position.x,tfp->pose.pose.position.y,tfp->pose.pose.position.z);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"tfReader");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1000,msgCallback3);
    ros::spin();
    return 0;
}