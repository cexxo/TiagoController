//AUTHOR: FRANCESCO CRISCI 2076739

#include"ros/ros.h"                                                                                                 //In this part of the code there are the necessary
#include "actionlib/client/simple_action_client.h"                                                                  //imports to allow the code to work. There are all 
#include "actionlib/client/terminal_state.h"                                                                        //the messages import, the import to allow the 
#include "controller/final_poseActionAction.h"                                                                      //comunication with the action server, and the import
#include <stdlib.h>                                                                                                 //to communicate with thehuman node.
#include <move_base_msgs/MoveBaseAction.h>                                                                          //
#include <geometry_msgs/PoseWithCovarianceStamped.h>                                                                //
#include "tiago_iaslab_simulation/Objs.h"                                                                           //
#include "assignment_2/detection.h"                                                                                 //
#include "assignment_2/move.h"                                                                                      //
#include "assignment_2/place.h"                                                                                     //

typedef actionlib::SimpleActionClient<controller::final_poseActionAction> actionClient;                             //This type allows us to use the action client.

//CONSTANTS DEFINITION                                                                                              //In this part of the code I define the constants we 
const std::vector<float> tiago_starting ={-6.58,1.37};                                                              //need. There is the Tiago starting position,
const std::vector<float> mark_1 = {1.93-tiago_starting[0],1.27-tiago_starting[1],-0.7};                             //and all the WayPoints tiago needs to go through 
const std::vector<float> markBlue ={1.41-tiago_starting[0],-0.85-tiago_starting[1],-0.68};                          //during the different routines.
const std::vector<float> markRed ={0.87-tiago_starting[0],-0.85-tiago_starting[1],-0.7};                            //
const std::vector<float> markGreen ={1.12-tiago_starting[0],-2.45-tiago_starting[1],0.7};                           //
const std::vector<float> markGreenShift ={1.12-tiago_starting[0]+0.85,-2.45-tiago_starting[1],-0.7};                //
const std::vector<float> markGreenMiddle = {6.5-tiago_starting[0],1.5-tiago_starting[1],0.7};                       //
const std::vector<float> markBlueDel ={5.9-tiago_starting[0],1.9-tiago_starting[1],-0.82};                          //
const std::vector<float> markGreenDel ={4.9-tiago_starting[0],0.45-tiago_starting[1],0.67};                         //
const std::vector<float> markRedDel ={3.85-tiago_starting[0],1.68-tiago_starting[1],-0.725};                        //

void moveTiago(actionlib::SimpleActionClient<controller::final_poseActionAction>* ac,float x, float y, float w){    //This function takes care of dealing with the actio
    ac->waitForServer();                                                                                            //server of the first assignment. It sends the pose
	controller::final_poseActionGoal goal;                                                                          //we want tiago to assume in the world reference 
    goal.pose.x = x;                                                                                                //frame. It deals also with the result and the 
    goal.pose.y = y;                                                                                                //success of the action.
    goal.pose.theta=w;                                                                                              //
    ac->sendGoal(goal);                                                                                             //
    bool finished_before_timeout = ac->waitForResult(ros::Duration(60.0));                                          //
    if(finished_before_timeout){                                                                                    //
        actionlib::SimpleClientGoalState state = ac->getState();                                                    //
    }                                                                                                               //
    else{                                                                                                           //
        ROS_ERROR("TIAGO FAILED THE NAVIGATION TO THE NEXT WAYPOINT...");                                           //
    }                                                                                                               //
}                                                                                                                   //

void sendMessage(ros::Publisher* pub,int obj_id, int msg_id, int max_iter){                                         //This function takes care of the communication with
    assignment_2::detection det_msg;                                                                                //the other nodes. We have two messages: one to
    assignment_2::place place_msg;                                                                                  //communicate that the detection cat start, and
    for(int i=0;(i < max_iter);i++){                                                                                //one to communicate that the placing routine can
        if(ros::ok()){                                                                                              //start. I have used a cpodification: if the msg_id
            if(msg_id==1){                                                                                          //is 1, send the detection message, otherwise send
                det_msg.obj=obj_id;                                                                                 //the place message.
                pub->publish(det_msg);                                                                              //We have set a tollerance of 100, which means that
                ros::spinOnce();                                                                                    //the message is gonna be sent 100 times.
            }                                                                                                       //
            else{                                                                                                   //
                place_msg.start=true;                                                                               //
                pub->publish(place_msg);                                                                            //
                ros::spinOnce();                                                                                    //
            }                                                                                                       //
        }                                                                                                           //
    }                                                                                                               //
}                                                                                                                   //

//IF IT IS THE FIRST OBJECT THAT TIAGO AS TO PICK, THE ROUTINE IS A BIT DIFFERENT, AT LEAST IN THE INITIAL PART
void routine(actionlib::SimpleActionClient<controller::final_poseActionAction>* ac, int id, bool isFirst){          //This is the core function of the node_a. 
    ros::NodeHandle h,k;                                                                                            //It first checks if it is the firt time this 
    ros::Publisher pub=h.advertise<assignment_2::detection>("detection",1000);                                      //function is called. If that is the case, tiago 
    ros::Publisher pub_place=h.advertise<assignment_2::place>("place",1000);                                        //moves through the corridor, and then moves to the
    boost::shared_ptr<assignment_2::move const> msg;                                                                //desired object. Otherwise, it movees directly to 
    const int max_iter = 100;                                                                                       //the desired object.
    if(isFirst){                                                                                                    //
        moveTiago(ac,mark_1[0],mark_1[1],mark_1[2]);                                                                //
    }                                                                                                               //
    switch (id)                                                                                                     //According to which object id we have, tiago 
    {                                                                                                               //executes one of the following routines.
    case 1:                                                                                                         //
        moveTiago(ac,markBlue[0],markBlue[1],markBlue[2]);                                                          //
        sendMessage(&pub,id,1,max_iter);                                                                            //
        ROS_INFO("WAITING FOR TIAGO TO PICK THE OBJECT");                                                           //
        msg=ros::topic::waitForMessage<assignment_2::move>("move",k);                                               //
        moveTiago(ac,markBlueDel[0],markBlueDel[1],markBlueDel[2]);                                                 //
        moveTiago(ac,markBlueDel[0],markBlueDel[1]-0.24,markBlueDel[2]);                                            //
        sendMessage(&pub_place,id,2,max_iter);                                                                      //
        ROS_INFO("WAITING FOR TIAGO TO PLACE THE OBJECT");                                                          //
        msg=ros::topic::waitForMessage<assignment_2::move>("move",k);                                               //
        moveTiago(ac,mark_1[0],mark_1[1],mark_1[2]);                                                                //
        break;                                                                                                      //
    case 2:                                                                                                         //
        moveTiago(ac,markGreenShift[0],markGreenShift[1],markGreenShift[2]);                                        //
        moveTiago(ac,markGreen[0],markGreen[1],markGreen[2]);                                                       //
        sendMessage(&pub,id,1,max_iter);                                                                            //
        ROS_INFO("WAITING FOR TIAGO TO PICK THE OBJECT");                                                           //
        msg=ros::topic::waitForMessage<assignment_2::move>("move",k);                                               //
        moveTiago(ac,markGreenDel[0],markGreenDel[1],markGreenDel[2]);                                              //
        sendMessage(&pub_place,id,2,max_iter);                                                                      //
        ROS_INFO("WAITING FOR TIAGO TO PLACE THE OBJECT");                                                          //
        msg=ros::topic::waitForMessage<assignment_2::move>("move",k);                                               //
        moveTiago(ac,markGreenMiddle[0],markGreenMiddle[1],markGreenMiddle[2]);                                     //
        moveTiago(ac,mark_1[0],mark_1[1],mark_1[2]);                                                                //
        break;                                                                                                      //
    case 3:                                                                                                         //
        moveTiago(ac,markRed[0],markRed[1],markRed[2]);                                                             //
        sendMessage(&pub,id,1,max_iter);                                                                            //
        ROS_INFO("WAITING FOR TIAGO TO PICK THE OBJECT");                                                           //
        msg=ros::topic::waitForMessage<assignment_2::move>("move",k);                                               //
        moveTiago(ac,markRedDel[0],markRedDel[1],markRedDel[2]);                                                    //
        sendMessage(&pub_place,id,2,max_iter);                                                                      //
        ROS_INFO("WAITING FOR TIAGO TO PLACE THE OBJECT");                                                          //
        msg=ros::topic::waitForMessage<assignment_2::move>("move",k);                                               //
        moveTiago(ac,mark_1[0],mark_1[1],mark_1[2]);                                                                //
        break;                                                                                                      //
    }                                                                                                               //
}                                                                                                                   //

void chooseRoutine(int id, bool isFirst){                                                                           //This function choose the routine according to the
    actionlib::SimpleActionClient<controller::final_poseActionAction> ac("final_poseAction",true);                  //ID and prints which routine tiago is gonna execute.
    if(id==1){                                                                                                      //
        ROS_INFO("GONNA DO BLUE ROUTINE");                                                                          //
    }                                                                                                               //
    else if(id==2){                                                                                                 //
        ROS_INFO("GONNA DO GREEN ROUTINE");                                                                         //
    }                                                                                                               //
    else{                                                                                                           //
        ROS_INFO("GONNA DO RED ROUTINE");                                                                           //
    }                                                                                                               //
    routine(&ac,id,isFirst);                                                                                        //
}                                                                                                                   //

int main(int argc, char** argv){                                                                                    //
    ros::init(argc,argv,"node_a");                                                                                  //Here we initialize ros parameters and other useful
    ros::NodeHandle n;                                                                                              //varaible.
    ros::Rate loop_rate(0.33);                                                                                      //
    std::vector<int> requestedObjects;                                                                              //
    bool alreadyRequested = false;                                                                                  //
    //FLAG THAT WILL TELL ME IF THE IT IS THE FIRST OBJECT TO PICK OR NO                                            //
    bool isFirst = true;                                                                                            //
    //HERE I'M CREATING THE SERVICE AND I'M SENDINGG IT TO THE HUMAN NODE                                           //
    ros::ServiceClient humanClient = n.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");          //
    tiago_iaslab_simulation::Objs humanService;                                                                     //
    //LET'S GO FOR NOW WITH JUST THE FIRST BOOL EQUAL TO TRUE                                                       //
    humanService.request.ready = true;                                                                              //
    humanService.request.all_objs = false;                                                                          //
    //CHECKS ON THE SERVICE STATUS
    while(ros::ok()){                                                                                               //We request the id to the human node.
        if(humanClient.call(humanService)){                                                                         //
            int temp_id = humanService.response.ids[0];                                                             //
            for(int k = 0; k<requestedObjects.size();k++){                                                          //We checke if the id we got was already asked.
                if(temp_id == requestedObjects[k])                                                                  //
                    alreadyRequested=true;                                                                          //
            }                                                                                                       //
            if(alreadyRequested){                                                                                   //If the Id was already requested, tiago is gonna 
                ROS_INFO("TIAGO ALREADY REQUESTED THIS OBJECT, GONNA REQUEST A NEW ONE IN A FEW SECONDS :)");       //wait 3 seconds before requesting a new id.
                alreadyRequested = false;                                                                           //
                loop_rate.sleep();                                                                                  //
                continue;                                                                                           //
            }                                                                                                       //
            requestedObjects.push_back(temp_id);                                                                    //
            chooseRoutine(temp_id,isFirst);                                                                         //Do the routine of the asked object.
            isFirst=false;                                                                                          //
            if(requestedObjects.size()==3)                                                                          //Once you have placed all the objects, stop 
                break;                                                                                              //requesting them.
        }                                                                                                           //
        else{                                                                                                       //
            ROS_ERROR("SOMETHING WENT WRONG WHILE CONTACTING THE HUMAN_NODE...");                                   //
        }                                                                                                           //
        ros::spinOnce();                                                                                            //
    }                                                                                                               //
    ROS_INFO("TIAGO FINISHED TO PICK ALL THE OBJECTS");                                                             //
    ros::shutdown();                                                                                                //
    return 0;                                                                                                       //
}