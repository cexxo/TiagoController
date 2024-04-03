//AUTHOR: GIOVANNI FRISO 2090300 & LORENZO GIANNINI 2091019

// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

// Message containing the pose of the pickable object
#include <apriltag_ros/AprilTagDetectionArray.h>

#include <tf/transform_listener.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Gazebo attacher
#include <gazebo_ros_link_attacher/Attach.h>

// Local message 
#include "assignment_2/pick.h"
#include "assignment_2/move.h"
#include "assignment_2/place.h"

// Geometric shapes to enlarge the collision objects
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

// Constant global variables
const float pi=3.14;
const std::vector<std::string> obj_shapes ={"tiago","Hexagon", "Triangle", "cube"};
const std::string frame="base_footprint"; // Frame name used several times in the code
const std::vector<std::string> ids = {"table", "blue_hexagon", "green_triangle"," red_cube", "gold_obs_0", "gold_obs_1", "gold_obs_2", "gold_obs_3", "cylinder", "wall"}; // Vector of names of the collision objects
// Collisions dimensions experimentally found
const std::vector<float> blue_hexagon={0.03, 0.1};
const std::vector<float> golden_hexagon={0.055, 0.3};
const std::vector<float> green_triangle={0.04, 0.05};
const float red_cube_edge=0.04;
const std::vector<float> table={1, 1.5, 1};
const std::vector<float> wall={2.5, 0.5, 2};
const std::vector<float> delivery={0.25, 0.8};
const std::vector<float> delivery_position={0.75, 0, 0.36, 0.75, 1};

// Auxiliary functions to shape the primitive
// Cylinder for blue hexagon, cylindrical bases and golden obstacles
// Cone for green triangle
// Box for table, red cube and wall
// INPUT: primitive to shape and its shape dimensions
// OUTPUT: none
void addCylinder(shape_msgs::SolidPrimitive* primitive, const float radius, const float height){
    primitive->type = shape_msgs::SolidPrimitive::CYLINDER;
    primitive->dimensions.resize(2);
    primitive->dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
    primitive->dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;  
}

void addCone(shape_msgs::SolidPrimitive* primitive, const float radius, const float height){
    primitive->type = shape_msgs::SolidPrimitive::CONE;
    primitive->dimensions.resize(2);
    primitive->dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = radius;
    primitive->dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = height;  
}

void addCube(shape_msgs::SolidPrimitive* primitive, const float depth, const float width, const float height){
    primitive->type = shape_msgs::SolidPrimitive::BOX;
    primitive->dimensions.resize(3);
    primitive->dimensions[shape_msgs::SolidPrimitive::BOX_X] = depth;
    primitive->dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
    primitive->dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height; 
}

// Function to add collision object to the planning scene
// INPUT: planning scene where Tiago works, id of the object to shape, id of the frame, pose of the object to shape
// OUTPUT: none
void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, const int id, const std::string frame_id, geometry_msgs::Pose& pose){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id=ids[id];
    shape_msgs::SolidPrimitive primitive;
    geometry_msgs::Pose& pose2=pose; 
    switch(id){
        case 1:
            addCylinder(&primitive, blue_hexagon[0], blue_hexagon[1]);
            break;
        case 2:
            addCone(&primitive, green_triangle[0], green_triangle[1]);
            pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,pi);
            break;
        case 3:
            addCube(&primitive, red_cube_edge, red_cube_edge, red_cube_edge);
            break;
        case 0:
            addCube(&primitive, table[0], table[1], table[2]);
            break;
        case 8:
            addCylinder(&primitive, delivery[0], delivery[1]);
            break;
        case 9:
            addCube(&primitive, wall[0], wall[1], wall[2]);
            break;
        default:
            addCylinder(&primitive, golden_hexagon[0], golden_hexagon[1]);
            break;
    }
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose2);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
    // If the following line is not present, the collision objects are not added to the planning scene
    ros::Duration(2).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects);
}

// Function to attach the object to the gripper
// INPUT: planning scene where Tiago works, id of the object to attach, given by the message "pick"
// OUTPUT: request gazebo_ros_link_attacher::Attach containing which models to attach
gazebo_ros_link_attacher::Attach attachObjectToGripper(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, const int id){
    moveit_msgs::AttachedCollisionObject attacher;
    gazebo_ros_link_attacher::Attach req;
    std::vector<std::string> object_ids;
    object_ids.push_back(ids[id]);
    planning_scene_interface.removeCollisionObjects(object_ids);
    attacher.object.id = ids[id];
    // Object has to be attached to the gripper (arm_7_link)
    attacher.link_name = "arm_7_link";
    attacher.object.operation = attacher.object.ADD;
    planning_scene_interface.applyAttachedCollisionObject(attacher);
    req.request.model_name_1 = obj_shapes[id];
    req.request.link_name_1 = obj_shapes[id]+"_link";
    req.request.model_name_2 = obj_shapes[0];
    req.request.link_name_2 = "arm_7_link";
    return req;
}

//Function to detach the object from the gripper
// INPUT: planning scene where Tiago works, id of the object to detach, given by the message "pick"
// OUTPUT: request gazebo_ros_link_attacher::Attach containing which models to detach
gazebo_ros_link_attacher::Attach detachObjectFromGripper(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, const int id){
    moveit_msgs::AttachedCollisionObject detachReq;
    gazebo_ros_link_attacher::Attach req;
    // Object has to be detached from the gripper (arm_7_link)
    detachReq.link_name = "arm_7_link";
    detachReq.object.operation = detachReq.object.REMOVE;
    planning_scene_interface.applyAttachedCollisionObject(detachReq); 
    req.request.model_name_1 = obj_shapes[id];
    req.request.link_name_1 = obj_shapes[id]+"_link";
    req.request.model_name_2 = obj_shapes[0];
    req.request.link_name_2 = "arm_7_link";
    return req;
}

// Function to open or close the gripper
// INPUT: float corresponding to the distance between the fingers (0.0 if closing, 1.0 if opening)
// OUTPUT: none
void move_gripper(float goal){
  // Action client subscribing to /parallel_gripper_controller/follow_joint_trajectory topic to move the gripper
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_ac("/parallel_gripper_controller/follow_joint_trajectory", true);
  gripper_ac.waitForServer();
  trajectory_msgs::JointTrajectory gripper_trajectory;
  gripper_trajectory.joint_names.push_back("arm_7_joint");
  trajectory_msgs::JointTrajectoryPoint point;
  // Final position definition
  point.positions.push_back(goal);
  point.time_from_start = ros::Duration(1.0);
  gripper_trajectory.points.push_back(point);
  control_msgs::FollowJointTrajectoryGoal gripper_goal;
  gripper_goal.trajectory = gripper_trajectory;
  gripper_ac.sendGoal(gripper_goal);
  gripper_ac.waitForResult();
  if (gripper_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Gripper moving goal achieved!");
  }
  else
  {
    ROS_ERROR("Failed to achieve gripper moving goal");
  }
}

// Function to move the arm when we give him joints positions instead of a pose
// INPUT: group of Tiago's joints to move
// OUTPUT: none
void execute_plan(moveit::planning_interface::MoveGroupInterface& move_group){
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        ROS_INFO("Planning successful. Moving the arm...");
        move_group.execute(my_plan);
    } else {
        ROS_WARN("Planning failed. The arm did not move.");
    }
} 

// Function to move the arm
// INPUT: Group of Tiago's joints to move, planning scene where Tiago works, pose of the object to pick, id of the same object, boolean to express in which phase we are (when it is true pick the object, else place the object)
// OUTPUT: none
void move(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, const geometry_msgs::Pose& pose, int id, bool flag){
    // All the following data are found through testing in Gazebo
    const std::vector<float> z_offset = {0.3, 0.163, 0.2345, 0.2075 ,0.75, 0.65}; // How to set the height of the arm in both picking and placing
    const float x_offset = 0.017; // Offset along x axis used for the green triangle (tag on an oblique surface)
    // Roll, pitch and yaw to place the gripper facing down
    const float roll = -0.011; 
    const float pitch = 1.57; 
    const std::vector<float> yaw = {0.037, 0.8};// The first value is the general case, the second specific for picking the grren triangle 
    //Joint values to reach intermidiate and safe pose
    const std::vector<double> intermidiate_pose = {0.0, 0.25, -0.3, 2, 1.7, -1.5, 0.0};
    const std::vector<double> secure_pose = {0.0, -1.5, -0.3, 2, 1.7, -1.3, 0.0}; 
    const std::vector<float> wall_position = {-0.2, -1.1, 0.300};
    geometry_msgs::Pose cylinder_pose = pose;
    // Setting parameters for the move_group
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setNumPlanningAttempts(20.0); 
    move_group.setPlanningTime(30.0); 
    move_group.setPlannerId("SBLkConfigDefault");
    move_group.setPoseReferenceFrame(frame);
    geometry_msgs::Pose target_pose = pose;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw[0]);
    ros::Rate rate(0.2);
    // If the object is the red cube, there is another collision object to add corresponding to the wall
      if(id == 3){
        geometry_msgs::Pose wall_pose;
        wall_pose.position.x=wall_position[0];
        wall_pose.position.y=wall_position[1];
        wall_pose.position.z=wall_position[2];
        addCollisionObject(planning_scene_interface, 9, frame, wall_pose);
      }
    if(!flag){
        //Placing phase
        move_group.setJointValueTarget(intermidiate_pose);
        execute_plan(move_group);
        target_pose.position.z=pose.position.z+z_offset[5];
        move_group.setPoseTarget(target_pose);
        move_group.setStartStateToCurrentState();
        move_group.move(); // Arm moved above the center of the shaped cylinder
        gazebo_ros_link_attacher::Attach detachReq = detachObjectFromGripper(planning_scene_interface, id);
        ros::service::call("/link_attacher_node/detach", detachReq); // Object detached by subscribing to the topic /link_attacher_node/detach
        move_gripper(1.0); // Gripper opened
        cylinder_pose.position.z=z_offset[4];
        addCollisionObject(planning_scene_interface, 1, frame, cylinder_pose);
    }
    else{
      //Picking phase
        target_pose.position.z=pose.position.z+z_offset[0];
        move_group.setPoseTarget(target_pose);
        move_group.setStartStateToCurrentState();
        move_group.move(); // Arm moved above the object to pick
        // If the object to pick is the green triangle, additional movements are required
        if(id == 2){
            target_pose.position.x += x_offset;
            target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw[1]); 
        }
        target_pose.position.z = pose.position.z+z_offset[id]; // Each object to pick has a different height to move the gripper in order to pick it up
        move_group.setPoseTarget(target_pose);
        move_group.setStartStateToCurrentState();
        moveit::planning_interface::MoveItErrorCode check = move_group.move();
        if (check == moveit::planning_interface::MoveItErrorCode::SUCCESS){
            gazebo_ros_link_attacher::Attach attachReq = attachObjectToGripper(planning_scene_interface, id);
            ros::service::call("/link_attacher_node/attach", attachReq); // Object attached by subscribing to the topic /link_attacher_node/attach
            move_group.setStartStateToCurrentState();
        }
        move_gripper(0.0); // Gripper closed
        target_pose.position.z=pose.position.z+z_offset[0]; 
        move_group.setPoseTarget(target_pose);
        move_group.setStartStateToCurrentState();
        rate.sleep();
        move_group.move(); // Arm moved above the center of the collision object of the object to pick
    }
    move_group.setJointValueTarget(intermidiate_pose);
    execute_plan(move_group);
    move_group.setJointValueTarget(secure_pose);
    execute_plan(move_group);
}

// Function to send the message
// INPUT: Publisher to communicate with node_a
// OUTPUT: none
void send_message(ros::Publisher* pub){
    ros::Rate loop_rate(10);
    int i=0;
    const int max_iter=10;
    while(i<max_iter && ros::ok()){
        assignment_2::move move_msg;
        move_msg.start=true;
        pub->publish(move_msg);
        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }
}

// Function to transform the coordinate from Gazebo coordinates
// INPUT: pose of the objectg to convert from Gazebo coordinates to the base_footprint frame
// OUTPUT: none
void trasform_from_Gazebo(geometry_msgs::Pose& pose){
    pose.position.x+=6.49;
    pose.position.y+=-1.37;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped pose_stamped;
    listener.waitForTransform("map", frame, ros::Time(0), ros::Duration(3.0));
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position = pose.position; 
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(1.57);
    geometry_msgs::PoseStamped pose_2;
    listener.transformPose(frame, pose_stamped, pose_2);
    pose_2.header.frame_id = frame;
    pose.position=pose_2.pose.position;
    pose.orientation=pose_2.pose.orientation;
}

// Function to get the position for the delivery cylinders
// INPUT: pose and id. The pose will become the delivery pillar's pose for object id in Gazebo coordinates
// OUTPUT: none
void get_pose_del(geometry_msgs::Pose& pose, int id){
    const std::vector<float> del = {1.02, 6.07, 5.07, 4.07, 0.35};//Coordinate from Gazebo
    pose.position.x = del[id];
    pose.position.y = del[0];
    pose.position.z = del[4];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_c");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(6); 
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("arm"); // Tiago's joints group to move corresponding to its arm
    const std::vector<float> table_positions = {1.25, -1.61, 0.300}; // Coordinates from Gazebo
    geometry_msgs::Pose table_pose; 
    geometry_msgs::Pose place;
    bool flag;
    for(int kk = 0; kk<3;kk++){
        flag=true;
        boost::shared_ptr<assignment_2::pick const> msg;
        msg = ros::topic::waitForMessage<assignment_2::pick>("pick", n);
        int id = msg->id;
        std::vector<apriltag_ros::AprilTagDetection> detections = msg->detections;
        // Adding collision objects
        for(int i = 0;i<detections.size();i++){
            addCollisionObject(planning_scene_interface, detections[i].id[0], detections[i].pose.header.frame_id, detections[i].pose.pose.pose);
        }
        // Transforming table coordinates
        table_pose.position.x = table_positions[0];
        table_pose.position.y = table_positions[1];
        table_pose.position.z = table_positions[2];
        trasform_from_Gazebo(table_pose);
        addCollisionObject(planning_scene_interface, 0, frame, table_pose);
        spinner.start();
        for(int i=0;i<detections.size();i++){
            if(detections[i].id[0] == id){
                move(move_group, planning_scene_interface, detections[i].pose.pose.pose, id, flag);      
            }
        }
        flag = false;
        spinner.stop();
        ros::Publisher pub=n.advertise<assignment_2::move>("move",1000);
        send_message(&pub); // The message to go to the delivery cylinder is published
        // After having picked an object, the planning scene is cleaned from all the previous collision objects
        std::vector<std::string> objects = planning_scene_interface.getKnownObjectNames(false);
        planning_scene_interface.removeCollisionObjects(objects);
        boost::shared_ptr<assignment_2::place const> msg_place;
        msg_place = ros::topic::waitForMessage<assignment_2::place>("place", n);
        spinner.start();
        // Transforming cylinders coordinates
        get_pose_del(place, id);
        trasform_from_Gazebo(place);
        // Adding collision objects
        addCollisionObject(planning_scene_interface, 8, frame, place);
        move(move_group, planning_scene_interface, place, id, flag);
        spinner.stop(); 
        objects = planning_scene_interface.getKnownObjectNames(false);
        planning_scene_interface.removeCollisionObjects(objects);
        send_message(&pub); // Communication with node_a
    }
    ros::spin();
    ros::shutdown();
    return 0;
}