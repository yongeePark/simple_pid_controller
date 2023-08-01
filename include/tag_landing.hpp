#ifndef __TAG_LANDING__
#define __TAG_LANDING__

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


#include <string>
#include <vector>
#include <array>

using namespace ros;
using namespace std;

enum MODE_LIST
{
	MODE_HOVER,
	MODE_FOLLOW,
	MODE_LAND,
};

bool is_pose_available = false;
double q[4];

void QuaternionToEuler(double& roll, double& pitch, double& yaw)
{	
    // roll (x-axis rotation)
    double t0 = +2.0 * (q[3] * q[0] + q[1] * q[2]);
    double t1 = +1.0 - 2.0 * (q[0] * q[0] + q[1]*q[1]);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (q[3] * q[1] - q[2] * q[0]);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = -std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    double t4 = +1.0 - 2.0 * (q[1]*q[1] + q[2] * q[2]);
    yaw = std::atan2(t3, t4);
}
void RegulateVelocity(double& vel, const double limit)
{
	if(abs(vel) > limit)
	{
		vel = vel / abs(vel) * limit;
	}
}

void RegulateAngle(double angle)
{
    while(abs(angle)>M_PI)
	{
		if(angle>0)
		{	angle = angle - 2 * M_PI;	}
		else if(angle<0)
		{	angle = angle + 2 * M_PI;	}
	}   
}


// Class define
class TagLandingController
{
private:
	NodeHandle nh_;
	NodeHandle nh_param_;
	Publisher pub_;
    	Publisher pub_goal_;
	Subscriber sub_;
	Subscriber sub_vel_;
	Subscriber sub_goal_;
	Subscriber sub_local_goal_;

	double current_position_[3];
	double current_attitude_[3];
	double vel_x_;
	double vel_y_;
	double vel_z_;
	geometry_msgs::PoseStamped localpose_;

	double goal_[3];
	// double last_yaw;
	double Kx_;
	double Ky_;
	double Kz_;
	double Kyaw_;

	bool use_yaw_;

	double velx_limit_;
	double vely_limit_;
	double velz_limit_;
	double yaw_limit_;

	double last_cmd_x_;
	double last_cmd_y_;

	double last_yaw_;

	int current_mode_;

    // these are variables for landing
    double landing_height_; // try landing when drone arrives to this height
    double hovering_height_;
    double landing_velz_limit_;
    bool is_landing_ready_;


public:
    void PrintInfo();
	double GetDistToGoal();
	double GetXYDistToGoal();
    void ReadROSParam();
    
	// constructor
	TagLandingController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param)
	:nh_(nh), nh_param_(nh_param), goal_{0.0,0.0,0.0},
    use_yaw_(false), is_landing_ready_(false),
    last_yaw_(0.0), last_cmd_x_(0.0), last_cmd_y_(0.0)
	{
		pub_            = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);
        pub_goal_       = nh_.advertise<geometry_msgs::PoseStamped>("/controller_goal",1);
		sub_            = nh_.subscribe("/mavros/local_position/pose",1,&TagLandingController::PoseCallback, this);
		sub_vel_        = nh_.subscribe("/mavros/local_position/velocity_local",1,&TagLandingController::VelCallback, this);
		sub_goal_       = nh_.subscribe("tag_detections/transformed_pose",1,&TagLandingController::GoalCallback, this);
		// sub_local_goal_ = nh_.subscribe("/apriltag_goal",1,&PIDController::LocalGoalCallback,this);

		current_mode_ = MODE_HOVER;
		
		vel_x_ = 0;
		vel_y_ = 0;
		vel_z_ = 0;
        
        

        // Read and assign ros parameter
		ReadROSParam();

        // set initial goal
        	goal_[2] = hovering_height_;
	}
	void PublishVelocity();
    
	void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
	// void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    void VelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg); 
    
	
};

// Set Current Position
// And publish velocity command
void TagLandingController::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{


    localpose_= *msg;

    current_position_[0] = localpose_.pose.position.x;	
    current_position_[1] = localpose_.pose.position.y;
    current_position_[2] = localpose_.pose.position.z;

    q[0] = localpose_.pose.orientation.x; 
    q[1] = localpose_.pose.orientation.y; 
    q[2] = localpose_.pose.orientation.z; 
    q[3] = localpose_.pose.orientation.w; 

    // wait until drone arrives to enough height
    if(current_position_[2] > landing_height_ && !is_landing_ready_)
    {   is_landing_ready_ = true;   }

    QuaternionToEuler(current_attitude_[0],current_attitude_[1],current_attitude_[2]);
    //PublishVelocity();
    is_pose_available = true;

    PublishVelocity();
}
void TagLandingController::ReadROSParam()
{
// Parameter Setting
    std::string nodename = "/tag_landing";
    if (!nh_param_.getParam(nodename+"/use_yaw",use_yaw_))
    {
        std::cout<<"[Warning] Please set [use_yaw_] parameter, default : false"<<std::endl;
        use_yaw_ = false;
    }
    if (!nh_param_.getParam(nodename+"/Kx",Kx_))
    {
        std::cout<<"[Warning] Please set [Kx] parameter, default : 1.0"<<std::endl;
        Kx_ = 1.0;
    }
    if (!nh_param_.getParam(nodename+"/Ky",Ky_))
    {
        std::cout<<"[Warning] Please set [Ky] parameter, default : 1.0"<<std::endl;
        Ky_ = 1.0;
    }
    if (!nh_param_.getParam(nodename+"/Kz",Kz_))
    {
        std::cout<<"[Warning] Please set [Kz] parameter, default : 0.8"<<std::endl;
        Kz_ = 0.8;
    }
    if (!nh_param_.getParam(nodename+"/Kyaw",Kyaw_))
    {
        std::cout<<"[Warning] Please set [Kyaw] parameter, default : 3.0"<<std::endl;
        Kyaw_ = 3.0;
    }
    if (!nh_param_.getParam(nodename+"/velx_limit",velx_limit_))
    {
        std::cout<<"[Warning] Please set [velx_limit] parameter, default : 0.4"<<std::endl;
        velx_limit_ = 0.4;
    }
    if (!nh_param_.getParam(nodename+"/vely_limit",vely_limit_))
    {
        std::cout<<"[Warning] Please set [vely_limit] parameter, default : 0.4"<<std::endl;
        vely_limit_ = 0.4;
    }
    if (!nh_param_.getParam(nodename+"/velz_limit",velz_limit_))
    {
        std::cout<<"[Warning] Please set [velz_limit] parameter, default : 0.2"<<std::endl;
        velz_limit_ = 0.2;
    }
    if (!nh_param_.getParam(nodename+"/yaw_limit",yaw_limit_))
    {
        std::cout<<"[Warning] Please set [yaw_limit] parameter, default : 0.15"<<std::endl;
        yaw_limit_ = 0.15;
    }
    if (!nh_param_.getParam(nodename+"/landing_height",landing_height_))
    {
        std::cout<<"[Warning] Please set [landing_height] parameter, default : 0.7"<<std::endl;
        landing_height_ = 0.7;
    }
    if (!nh_param_.getParam(nodename+"/hovering_height",hovering_height_))
    {
        std::cout<<"[Warning] Please set [hovering_height] parameter, default : 2.0"<<std::endl;
        hovering_height_ = 2.0;
    }
    if (!nh_param_.getParam(nodename+"/landing_velz_limit",landing_velz_limit_))
    {
        std::cout<<"[Warning] Please set [landing_velz_limit] parameter, default : 0.5"<<std::endl;
        landing_velz_limit_ = 2.0;
    }
    std::cout<<"================================="<<std::endl;
    std::cout<<"Parameters"<<std::endl
    <<"use_yaw_ : "<< (use_yaw_ ? "true" : "false")<<endl
    <<"Kx_     : "<<Kx_<<endl
    <<"Ky_     : "<<Ky_<<endl
    <<"Kz_      : "<<Kz_<<endl
    <<"Kyaw_    : "<<Kyaw_<<endl;
    std::cout<<"================================="<<std::endl;
}

// Print curret information
void TagLandingController::PrintInfo()
{
    std::cout<<"Current Pose x y z : "<<current_position_[0]<<", "<<current_position_[1]<<", "<<current_position_[2]<<std::endl;
    std::cout<<"Current Goal x y z : "<<goal_[0]<<", "<<goal_[1]<<", "<<goal_[2]<<std::endl;
    std::cout<<"Current MODE : "<<current_mode_<<std::endl;
}

// Get Distance to goal from current position
double TagLandingController::GetDistToGoal()
{
    return sqrt(pow(goal_[0] - current_position_[0] , 2)
        + pow(goal_[1] - current_position_[1] , 2)
        + pow(goal_[2] - current_position_[2] , 2));
}
// Get distance to goal from current position in x-y plane
double TagLandingController::GetXYDistToGoal()
{
    return sqrt(pow(goal_[0] - current_position_[0] , 2)
        + pow(goal_[1] - current_position_[1] , 2));
}

    
#endif
