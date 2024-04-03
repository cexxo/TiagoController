#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "controller/obstacle_msg.h"
#include <cmath>

//Class to better handle the points in cartesian coordinates.
class Point{
    protected:
        float x;
        float y;
    public:
    Point(float i, float j){
        x=i;
        y=j;
    }
    float get_x() const {return x;}
    float get_y() const {return y;}
};

//Global vectors we wanted to use for the feedback.
std::vector<float> x_centers;
std::vector<float> y_centers;

//The idea behind is to check if every point has distance from the center equal to r.
bool check_circle(const std::vector<Point> block, const Point center, const float radius)
{
	//Inside the following for cicle the distance between the center and the points inserted in the possible arc of circumference is computed.
	int discarded = 0;
	float offset = 0.02; // Sperimental numbers
	int max = 1;
	for(int i = 0; i < block.size(); i++)
	{
		if(discarded > max) {return false;}
		float x_coor = pow(center.get_x()-block[i].get_x(),2);
		float y_coor = pow(center.get_y()-block[i].get_y(),2);
		float distance = sqrt(x_coor+y_coor);
		if(distance <= radius - offset || distance >= radius + offset)
		{discarded ++;}
	}
	
	
	return true;
}

//Check if in the block found, taking the first, the middle and last point, a circle can be inscribed.
//The idea is to find the middle points between the 3 points and using the perpendicular line going throught them to find the center.
void find_circle(const std::vector<Point> block, std::vector<Point> centers, const float min_r, const float max_r){
    Point first=block[0];
    Point middle=block[(block.size()-1)/2];
    Point last=block[block.size()-1];

    //Find the middle points.
    float tmp_x=(first.get_x()+middle.get_x())/2;
    float tmp_y=(first.get_y()+middle.get_y())/2;
    Point p1(tmp_x,tmp_y);

    tmp_x=(middle.get_x()+last.get_x())/2;
    tmp_y=(middle.get_y()+last.get_y())/2;
    Point p2(tmp_x,tmp_y);

    //Find the slopes.
    float m1=(middle.get_y()-first.get_y())/(middle.get_x()-first.get_x());
    float m2=(last.get_y()-middle.get_y())/(last.get_x()-middle.get_x());

    //Check for vertical lines.
    if(std::isinf(m1)){
        m1=-m2;
    } else if(std::isinf(m2)){
        m2=-m1;
    } else{
        //If ok, take the perpendicular slopes.
        m1=-1/m1;
        m2=-1/m2;
    }

    //Solve the system of eq.
    //Eq1->y1-Y=-(1/m1)(x1-X)
    //Eq2->y2-Y=-(1/m2)(x2-X)
    float x=((p2.get_y()-p1.get_y())-(m2*p2.get_x()-m1*p1.get_x()))/(m1-m2);
    float y=m1*x-m1*p1.get_x()+p1.get_y();

    //Find the radius.
    float r=sqrt(pow(x-first.get_x(),2)+pow(y-first.get_y(),2));

    //check if it is accettable
    if(r>=min_r && r<=max_r){
        Point c(x,y);
        //Discard the concave circle.
	    float mid_range = sqrt(middle.get_x()*middle.get_x() + middle.get_y()*middle.get_y());
	    float first_range = sqrt(first.get_y()*first.get_y() + first.get_x()*first.get_x());
	    float last_range = sqrt(last.get_y()*last.get_y() + last.get_x()*last.get_x());
        if(first_range < mid_range && last_range < mid_range){
            return;
        }
        if(check_circle(block, c, r)){
        	//ROS_INFO("Circle with radius %f", r);
		    ROS_INFO("Tiago has found an obstacle at [%f, %f] w.r.t. its own frame", c.get_x(), c.get_y());
        	x_centers.push_back(c.get_x());
        	//ROS_INFO("x_centers %f %f",c.get_x(),x_centers[x_centers.size()-1]);
        	y_centers.push_back(c.get_y());
        	//ROS_INFO("y_centers %f %f",c.get_y(),y_centers[y_centers.size()-1]);
        }
    }
}

void msgCallback(const sensor_msgs::LaserScanPtr& ls){              
	float range = (ls->angle_max - ls->angle_min)/ls->angle_increment;
	//ROS_INFO("# of scanned points %f", range);                                                          
	int max_blocks = range;
	std::vector<Point> points; 
	std::vector<Point> centers;
	std::vector<std::vector<Point>> blocks(max_blocks); 
	const int blind_spots = 20;                                   
	const float gap = 0.1;
	const float min_r = 0.17;
	const float max_r = 0.19;
	float tmp_x=0.0;
	float tmp_y=0.0;
	const int min_p = 3;
	x_centers.clear();
	y_centers.clear();
    //Data reading and creation of blocks
    int bp=0;
	for(int i = blind_spots; i < range-blind_spots-1;i++){              
		if(ls->ranges[i] != 0 && ls->ranges[i] <= ls->range_max){            
            tmp_x=ls->ranges[i]*cos(ls->angle_min+i*ls->angle_increment);                         
            tmp_y=ls->ranges[i]*sin(ls->angle_min+i*ls->angle_increment); 
            Point tmp(tmp_x,tmp_y);         
            blocks[bp].push_back(tmp);
            if(abs(ls->ranges[i]-ls->ranges[i+1])>=gap){
                    bp++;
            }
	    }
    }
  

    for(int i=0;i<bp;i++){
    	if(blocks[i].size()<min_p){continue;}
       	find_circle(blocks[i], centers, min_r, max_r);
    }
   
}

int main(int argc, char **argv){
 ros::init(argc,argv, "laser_scanner");
 ros::NodeHandle n;  
 //We subscribe to the /scan_raw topic in order
 controller::obstacle_msg msg;
 ros::Subscriber sub = n.subscribe("/scan_raw", 1000, msgCallback);
 msg.obstacles_x = x_centers;
 msg.obstacles_y = y_centers;
 ros::Publisher scanned_obstacles = n.advertise<controller::obstacle_msg>("scanned_obstacles", 1000);
 scanned_obstacles.publish(msg);
 ros::spin();
 return 0;
}