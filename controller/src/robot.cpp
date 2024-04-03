//This receives a call from the controller (server) and computes the position of the obstacles in order to achieve the final pose avoiding obstacles
//It gives the feedback to the controller
// After the task finishes it gives the position of the obstacles to the server


//ROS INCLUDE
#include"ros/ros.h"

//ROS SERVER AND CLIENT
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

//MOVE BASE INCLUDES
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "controller/final_poseActionAction.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "sensor_msgs/LaserScan.h"
#include "controller/obstacle_msg.h"

//C++ UTILITIES
#include <sstream>
#include<string>

//DEFINITION OF THE MOVE BASE TYPE TO MOVE THE ROBOT
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionServer<controller::final_poseActionAction> actionServer;
void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback,actionServer* as_);


class final_poseActionAction{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<controller::final_poseActionAction> as_;
		std::string action_name_;
		controller::final_poseActionFeedback feedback_;
		controller::final_poseActionResult result_;
	public:
		final_poseActionAction(std::string name) : as_(nh_,name,boost::bind(&final_poseActionAction::executeCB,this,_1),false),action_name_(name){
		as_.start();
	}
	//Function that coordinates the navigation.
	void navController(float x, float y, float w){
		MoveBaseClient ac("move_base", true);
		move_base_msgs::MoveBaseGoal tiagoGoal;
		const float correctionFactor = 1.0;//It avoids having not valid quaternions.
		while(!ac.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting to send the goal [%f,%f,%f] to the move_base_server...",x,y,w);
		}
		tiagoGoal.target_pose.header.frame_id = "map";
		tiagoGoal.target_pose.pose.position.x = x;
		tiagoGoal.target_pose.pose.position.y = y ;
		if(w<0)tiagoGoal.target_pose.pose.orientation.z =-w;
		else tiagoGoal.target_pose.pose.orientation.z = w;
		tiagoGoal.target_pose.pose.orientation.w = w;
		ROS_INFO("Tiago has the following parameters: FRAME_ID: [%s]; GOAL_X: [%f]; GOAL_Y:[%f]; ORIENTATION: [%f]",tiagoGoal.target_pose.header.frame_id.c_str(),tiagoGoal.target_pose.pose.position.x,tiagoGoal.target_pose.pose.position.y,tiagoGoal.target_pose.pose.orientation.w);
		ROS_INFO("SENDING THE GOAL TO THE move_base_server");
		ac.sendGoal(tiagoGoal,MoveBaseClient::SimpleDoneCallback(), MoveBaseClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1,&as_));
		feedback_.status = "STARTED NAVIGATING";
		as_.publishFeedback(feedback_);
		ac.waitForResult();
		feedback_.status = "FINISHED NAVIGATING";
		as_.publishFeedback(feedback_);
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("TIAGO SUCCESFULLY NAVIGATED TO THE DESIRED POSITION");
		else
			ROS_ERROR("TIAGO FAILED THE NAVIGATION");
	}


	//Core of the action server
	void executeCB(const controller::final_poseActionGoalConstPtr &goal){
		move_base_msgs::MoveBaseGoal tiagoGoal;
		bool success = true;
		for(int i=0;i<1;i++){
			if(as_.isPreemptRequested() || !ros::ok()){
				ROS_INFO("%s: Preempted",action_name_.c_str());
				as_.setPreempted();
				success = false;
			}
			navController(goal->pose.x,goal->pose.y,goal->pose.theta);
		}
		if(success){
			result_.goal_reached = true;
			ROS_INFO("%s, succedeed",action_name_.c_str());
			as_.setSucceeded(result_);
		}
	}
	~final_poseActionAction(void){}
};

//Function to end back the feedback on Tiago status. It updates the current position of tiago and its status.
void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback,actionServer* as_){
	controller::final_poseActionFeedback feedback_;
	feedback_.current_x = feedback->base_position.pose.position.x;
	feedback_.current_y = feedback->base_position.pose.position.y;
	feedback_.status = "IS NAVIGATING";
	as_->publishFeedback(feedback_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot");
	final_poseActionAction final_pose("final_poseAction");
	ros::spin();
	return 0;
}