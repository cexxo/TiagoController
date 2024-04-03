// This receives the final pose from the user (client)

#include"ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "controller/final_poseActionAction.h"
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<controller::final_poseActionAction> actionClient;

void feedbackCallback(const controller::final_poseActionFeedbackConstPtr& feedback)
{
	ROS_INFO("TIAGO %s AND  IT IS CURRENTLY HERE: [%f,%f]",feedback->status.c_str(), feedback->current_x, feedback->current_y);
}


//We create an action client that will send the pose to the action server. We do a check on the parameters given in input and then it sends the goal.
//It waits around 10 second in the beginning so that Tiago can close its own arm.


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	if (argc!=4){ROS_ERROR("WRONG NUMBER OF PARAMETERS"); return 0;}
	actionlib::SimpleActionClient<controller::final_poseActionAction> ac("final_poseAction",true);
	ROS_INFO("Waiting for action server to start");
	ac.waitForServer();
	ROS_INFO("Action server started,sending goal.");
	controller::final_poseActionGoal goal;
	if(argv[1][0]=='-')
		goal.pose.x = atof(argv[1])*(-1)*(-1);
	else	
		goal.pose.x = atof(argv[1]);
	if(argv[2][0]=='-')
		goal.pose.y = atof(argv[2])*(-1)*(-1);
	else	
		goal.pose.y = atof(argv[2]);
	if(argv[3][0]=='-')
		goal.pose.theta = atof(argv[3])*(-1)*(-1);
	else	
		goal.pose.theta = atof(argv[3]);
	const float estimatedWaitingTuckTime = 0.1;//IT WAS 0.045
	controller::final_poseActionFeedback feedback;
	ros::Rate waiting(estimatedWaitingTuckTime);
	ROS_INFO("WAITING FOR TIAGO TO FOLD ITS ARM");
	waiting.sleep();
	ROS_INFO("TIAGO CAN FINALLY MOVE SAFELY :)");
	ac.sendGoal(goal, actionClient::SimpleDoneCallback(), actionClient::SimpleActiveCallback(), boost::bind(&feedbackCallback, _1));
	bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
	if(finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}
}