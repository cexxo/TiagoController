//AUTHOR: LORENZO GIANNINI 2091019

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "assignment_2/detection.h"
#include "assignment_2/pick.h"
//AprilTag
#include "apriltag_ros/AprilTagDetectionArray.h"
//TF
#include "tf/transform_listener.h"

//Global variable to set the velocities for the joints movement.
const float velocity=0.1;

//Function to lift the torso. It uses a SimpleActionClient to send the goal to the follow_joint_trajectory.
void liftTorso(float h){
    //Calls ActionClient.
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_controller("/torso_controller/follow_joint_trajectory", true);
    torso_controller.waitForServer(ros::Duration(3.0));
    //Generate the desired positions.
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(1);
    goal.trajectory.points[0].velocities.resize(1);
    goal.trajectory.points[0].positions[0]=h;
    goal.trajectory.points[0].velocities[0]=velocity;
    goal.trajectory.points[0].time_from_start=ros::Duration(1.0);
    //Send the positions.
    torso_controller.sendGoal(goal);
    bool timeout=torso_controller.waitForResult(ros::Duration(30.0));
    if(timeout){
        ROS_INFO("Torso moved");
    }else{
        ROS_ERROR("Torso movement did not finish in time");
    }
}

//Function to move the head. It uses a SimpleActionClient to send the goal to the follow_joint_trajectory. The values have been found sperimentally.
void moveHead(int id){
    //All head positions and velocities.
    const std::vector<float> head_blue={-0.25,-1.5};
    const std::vector<float> head_green={-0.25,-0.75};
    const std::vector<float> head_red={0.6,-0.8};
    const std::vector<float> head_default={0.0,0.0};
    //Calls ActionClient.
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_controller("/head_controller/follow_joint_trajectory", true);
    head_controller.waitForServer(ros::Duration(3.0));
    //Generate the desired positions.
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("head_1_joint");
    goal.trajectory.joint_names.push_back("head_2_joint");
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].velocities.resize(2);
    //First joint is z axis rotation, second joint x axis.
    switch(id){
        case 1:
            goal.trajectory.points[0].positions[0]=head_blue[0];
            goal.trajectory.points[0].positions[1]=head_blue[1];
            break;
        case 2:
            goal.trajectory.points[0].positions[0]=head_green[0];
            goal.trajectory.points[0].positions[1]=head_green[1];
            break;
        case 3:
            goal.trajectory.points[0].positions[0]=head_red[0];
            goal.trajectory.points[0].positions[1]=head_red[1];
            break;
        case 0:
            goal.trajectory.points[0].positions[0]=head_default[0];
            goal.trajectory.points[0].positions[1]=head_default[1];
            break;
    }
    goal.trajectory.points[0].velocities[0]=velocity;
    goal.trajectory.points[0].velocities[1]=velocity;
    goal.trajectory.points[0].time_from_start=ros::Duration(2.0);
    //send the positions.
    head_controller.sendGoal(goal);
    bool timeout=head_controller.waitForResult(ros::Duration(30.0));
    if(timeout){
        ROS_INFO("Head moved");
    }else{
        ROS_ERROR("Head movement did not finish in time");
    }
}

//Function to transform the pose in reference to the desired frame.
apriltag_ros::AprilTagDetection get_detections(std::string frame, apriltag_ros::AprilTagDetection detection){
    tf::TransformListener listener(ros::Duration(10.0));
    geometry_msgs::PoseStamped ps;
    geometry_msgs::PoseStamped final_pose;
    apriltag_ros::AprilTagDetection transformed_det;
    try{
        listener.waitForTransform(frame, detection.pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
        //from PoseWithCovariance to PoseStamped
        ps.header.frame_id=detection.pose.header.frame_id;
        ps.pose=detection.pose.pose.pose;
        //transform the coordiantes
        listener.transformPose(frame, ps, final_pose);
        //from PoseStamped to PoseWithCovarianceStamped
        transformed_det.id=detection.id;
        transformed_det.size=detection.size;
        transformed_det.pose.pose.pose=final_pose.pose;
        transformed_det.pose.pose.covariance.fill(0.0);
        transformed_det.pose.header.frame_id=final_pose.header.frame_id;
    }catch(tf::TransformException& ex){
        ROS_ERROR("Transformation error: %s", ex.what());
    }   
    return transformed_det; //returns a default value if there is an error for now.
}


//Function to communicate with moving_arm.
void send_message(ros::Publisher* pub, int id, std::vector<apriltag_ros::AprilTagDetection> buffer){
    ros::Rate loop_rate(10);
    int i=0;
    int max_times=10;
    while(i<max_times && ros::ok()){
        assignment_2::pick pick_msg;
        pick_msg.detections=buffer;
        pick_msg.id=id;
        pub->publish(pick_msg);
        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh_;

    //height for the torso to see better the table and avoid self collision. Found sperimentally.
    const float height=1.3;
    const int max_iter=3;
    const std::string frame="base_footprint";

    std::vector<apriltag_ros::AprilTagDetection> final_detections;
    int iter=0;
    bool flag=true;

    while(iter<max_iter){
        //Wait for Message from node_a to start the detection.
        boost::shared_ptr<assignment_2::detection const> msg=ros::topic::waitForMessage<assignment_2::detection>("detection",nh_);
        int id=msg->obj;
        //Lift the torso to see the table only on the first iteration
        if(flag){
            liftTorso(height);
            flag = false;
        }
        //Move the head of tiago to see the objects.
        moveHead(id);
        //AprilTag Detections
        boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> det=ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections",nh_);
        for(int i=0;i<det->detections.size();i++){
            final_detections.push_back(get_detections(frame, det->detections[i]));
        }
        ROS_INFO("Tiago has detected %d objects",final_detections.size());
        //Writing and sending te new message for moving_arm.
        ros::Publisher pub=nh_.advertise<assignment_2::pick>("pick",1000);
        send_message(&pub, id, final_detections);
        final_detections.clear();
        //Bringing the head on default position, so that the arm does not collide with it.
        moveHead(0);
        iter++;
    }
    ros::spin();
    ros::shutdown();
    return 0;
}