#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <vector>
#include <array>

#include <selfie_mission_planner/VisualTracking.h>

#define RadToDeg 180/M_PI


using namespace ros;
using namespace std;

enum MODE_LIST 
{
	TAKEOFF,
	TRACK,
	HOLD,
};
namespace SELFIE_STATUS
{
    enum
    {
        WAIT = 0,
        Tracking,

        too_many = 99
    };
}
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

// Class define
class PIDController
{
private:
	NodeHandle nh_;
	NodeHandle nh_param_;
	Publisher pub_;
	Subscriber sub_;
	Subscriber sub_goal_;
	Subscriber sub_mission_;

	double current_position_[3];
	double hold_position_[3];
	double current_attitude_[3];
    double hold_yaw_;
	geometry_msgs::PoseStamped localpose_;

	double goal_[3];
    double goal_yaw_;

	double Kxy_;
	double Kz_;
	double Kyaw_;

	bool use_yaw_;

	double velxy_limit_;
	double velz_limit_;
	double yaw_limit_;

	// for selfie drone
    double desired_subject_ratio_;

    double subject_ratio_thres_;
    double lateral_thres_; 

    double subject_ratio_;
    double lateral_offset_;
    
	double ref_x_ratio_;
	double ref_y_ratio_;
    double ref_yaw_ratio_;
    double selfie_height_;

	int mode_;

public:
	// constructor
	PIDController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param)
	:nh_(nh), nh_param_(nh_param), goal_{0.0,0.0,0.0}, goal_yaw_(0), use_yaw_(false),
        subject_ratio_(0), lateral_offset_(0)
	{
		pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/scout/mavros/setpoint_velocity/cmd_vel",1);
		sub_ = nh_.subscribe("/scout/mavros/local_position/pose",1,&PIDController::PoseCallback, this);
		//sub_goal_ = nh_.subscribe("/photo_zone_pose",1,&PIDController::GoalCallback, this);
		sub_mission_ = nh_.subscribe("/selfie_tracking",1,&PIDController::MissionCallback, this);

		mode_ = MODE_LIST::TAKEOFF;

		// Parameter Setting
		if (!nh_param_.getParam("use_yaw",use_yaw_))
        {
            std::cout<<"[Warning] Please set [use_yaw_] parameter, default : false"<<std::endl;
            use_yaw_ = false;
        }
		if (!nh_param_.getParam("Kxy",Kxy_))
        {
            std::cout<<"[Warning] Please set [Kxy] parameter, default : 1.0"<<std::endl;
            Kxy_ = 1.0;
        }
		if (!nh_param_.getParam("Kz",Kz_))
        {
            std::cout<<"[Warning] Please set [Kz] parameter, default : 0.8"<<std::endl;
            Kz_ = 0.8;
        }
		if (!nh_param_.getParam("Kyaw",Kyaw_))
        {
            std::cout<<"[Warning] Please set [Kyaw] parameter, default : 3.0"<<std::endl;
            Kyaw_ = 3.0;
        }
		if (!nh_param_.getParam("velxy_limit",velxy_limit_))
        {
            std::cout<<"[Warning] Please set [velxy_limit] parameter, default : 0.4"<<std::endl;
            velxy_limit_ = 0.4;
        }
		if (!nh_param_.getParam("velz_limit",velz_limit_))
        {
            std::cout<<"[Warning] Please set [velz_limit] parameter, default : 0.2"<<std::endl;
            velz_limit_ = 0.2;
        }
		if (!nh_param_.getParam("yaw_limit",yaw_limit_))
        {
            std::cout<<"[Warning] Please set [yaw_limit] parameter, default : 0.15"<<std::endl;
            yaw_limit_ = 0.15;
        }
        // paramerter for selfie drone
		if (!nh_param_.getParam("desired_subject_ratio",desired_subject_ratio_))
        {
            std::cout<<"[Warning] Please set [desired_subject_ratio] parameter, default : 0.1"<<std::endl;
            desired_subject_ratio_ = 0.1;
        }
		if (!nh_param_.getParam("subject_ratio_thres",subject_ratio_thres_))
        {
            std::cout<<"[Warning] Please set [subject_ratio_thres] parameter, default : 0.05"<<std::endl;
            subject_ratio_thres_ = 0.05;
        }
		if (!nh_param_.getParam("lateral_thres",lateral_thres_))
        {
            std::cout<<"[Warning] Please set [lateral_thres] parameter, default : 0.05"<<std::endl;
            lateral_thres_= 0.1;
        }
		if (!nh_param_.getParam("ref_x_ratio",ref_x_ratio_))
        {
            std::cout<<"[Warning] Please set [ref_x_ratio] parameter, default : 0.1"<<std::endl;
            ref_x_ratio_ = 0.1;
        }
		if (!nh_param_.getParam("ref_y_ratio",ref_y_ratio_))
        {
            std::cout<<"[Warning] Please set [ref_y_ratio] parameter, default : 0.1"<<std::endl;
            ref_y_ratio_ = 0.1;
        }
		if (!nh_param_.getParam("ref_yaw_ratio",ref_yaw_ratio_))
        {
            std::cout<<"[Warning] Please set [ref_yaw_ratio] parameter, default : 0.05"<<std::endl;
            ref_yaw_ratio_ = 0.05;
        }
		if (!nh_param_.getParam("selfie_height",selfie_height_))
        {
            std::cout<<"[Warning] Please set [selfie_height] parameter, default : 1.0"<<std::endl;
            selfie_height_ = 1.0;
        }
		std::cout<<"================================="<<std::endl;
		std::cout<<"Parameters"<<std::endl
		<<"use_yaw_ : "<< (use_yaw_ ? "true" : "false")<<endl
		<<"Kxy_     : "<<Kxy_<<endl
		<<"Kz_      : "<<Kz_<<endl
		<<"Kyaw_    : "<<Kyaw_<<endl
		<<"mode     : "<<mode_<<endl
		<<"selfie_height : "<<selfie_height_<<endl;
		std::cout<<"================================="<<std::endl;
	}
	int GetMode()
	{
		return mode_;
	}
	void PublishVelocity()
	{
		switch (mode_) {
			case MODE_LIST::TAKEOFF:
                goal_[0]  = 0; 
                goal_[1]  = 0; 
                goal_[2]  = selfie_height_; 
                goal_yaw_ = 0;
		    	break;	
            case MODE_LIST::TRACK:
                {
                // 1. x_difference
                // if x_difference is bigger than x_desired,
                // the drone should go back
                // vice versa
                double x_diff = (desired_subject_ratio_- subject_ratio_) * ref_x_ratio_; 
                //x_diff = abs(x_diff) < 0.05 ? 0 : x_diff;
                goal_[0]  = x_diff +  current_position_[0];

                // 2. y_difference
                // when target is left, y_difference is bigger than 0
                // if target is on the left, drone should go a little left
                double y_diff = (lateral_offset_- 0) * ref_y_ratio_;
                //y_diff = abs(y_diff) < 0.05 ? 0 : y_diff;
                goal_[1] = y_diff + current_position_[1];

                // 3. altitude
                // the drone should maintain desired attitude
                goal_[2] = selfie_height_;

                // 4. yaw angle
                // when target is left, it should rotate postiively
                double yaw_diff = (lateral_offset_ - 0) * ref_yaw_ratio_;
                //yaw_diff = abs(yaw_diff) < 0.05 ? 0 : yaw_diff;
                goal_yaw_  = yaw_diff + current_attitude_[2];
                break;
                }
            case MODE_LIST::HOLD:
                goal_[0]  = hold_position_[0];
                goal_[1]  = hold_position_[1];
                goal_[2]  = hold_position_[2];
                goal_yaw_ = hold_yaw_;
                break;
		}


		//calculate linear velocity
		double cmd_x = (goal_[0] - current_position_[0]) * Kxy_;
		double cmd_y = (goal_[1] - current_position_[1]) * Kxy_;
		double cmd_z = (goal_[2] - current_position_[2]) * Kz_;
		RegulateVelocity(cmd_x,velxy_limit_);
		RegulateVelocity(cmd_y,velxy_limit_);
		RegulateVelocity(cmd_z,velz_limit_);

		// current yaw angle
		double current_yaw = current_attitude_[2];
		//double goal_yaw = std::atan2(goal_[1]-current_position_[1],goal_[0]-current_position_[0]);
		double goal_yaw = goal_yaw_; 
		//double goal_yaw = 0; 
		double yaw_diff = goal_yaw - current_yaw;

		// regulate yaw angle
		while(abs(yaw_diff)>M_PI)
		{
			if(yaw_diff>0)
			{	yaw_diff = yaw_diff - 2 * M_PI;	}
			else if(yaw_diff<0)
			{	yaw_diff = yaw_diff + 2 * M_PI;	}
		}

		
		double cmd_r = yaw_diff * Kyaw_;
		RegulateVelocity(cmd_r,yaw_limit_);
		
		geometry_msgs::TwistStamped command_msg;
		command_msg.header.stamp = ros::Time::now();


		/*
		if (abs(yaw_diff * RadToDeg) > 30 && GetXYDistToGoal() > 0.3
			&& current_position_[2] > 0.3)
		{
		    cmd_x = cmd_x / 5;
		    cmd_y = cmd_y / 5;
		}
		*/
		if ( current_position_[2] < 0.3)
		{   cmd_r = 0;	}

		// assign
		command_msg.twist.linear.x = cmd_x;
		command_msg.twist.linear.y = cmd_y; 
		command_msg.twist.linear.z = cmd_z; 

		command_msg.twist.angular.z = cmd_r; 
		pub_.publish(command_msg);
	}
	void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		localpose_= *msg;

		current_position_[0] = localpose_.pose.position.x;	
		current_position_[1] = localpose_.pose.position.y;
		current_position_[2] = localpose_.pose.position.z;

		q[0] = localpose_.pose.orientation.x; 
		q[1] = localpose_.pose.orientation.y; 
		q[2] = localpose_.pose.orientation.z; 
		q[3] = localpose_.pose.orientation.w; 

		QuaternionToEuler(current_attitude_[0],current_attitude_[1],current_attitude_[2]);
		//PublishVelocity();
		is_pose_available = true;
	}
	void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
	{
		goal_[0] = msg->pose.position.x;
		goal_[1] = msg->pose.position.y;
		goal_[2] = msg->pose.position.z;
	}
    void MissionCallback(const selfie_mission_planner::VisualTracking::ConstPtr& msg)
    {
        if(current_position_[2] > 0.7)
        {
        // 1. mission is tracking
        if(msg->modes == SELFIE_STATUS::Tracking)
        {
            // 2. if it is enough close
            // just set mode to hold
            double subject_ratio_diff = msg->subject_ratio - desired_subject_ratio_;
            if(abs(subject_ratio_diff) < subject_ratio_thres_ && 
                    abs(msg->lateral_offset) < lateral_thres_)
            {
                if(mode_ != MODE_LIST::HOLD)
                {
                    hold_position_[0] = current_position_[0];
                    hold_position_[1] = current_position_[1];
                    hold_position_[2] = current_position_[2];
                    hold_yaw_ = current_attitude_[2];
                }
                mode_ == MODE_LIST::HOLD;
            }
            //set mode to track
            else
            {
                mode_ = MODE_LIST::TRACK;
                subject_ratio_  = msg->subject_ratio;
                lateral_offset_ = msg->lateral_offset;
            }
        }
        // if command is wait or too many,
        // go to hold mode
        else if(msg->modes == SELFIE_STATUS::WAIT || msg->modes == SELFIE_STATUS::too_many)
        {
                mode_ == MODE_LIST::HOLD;
                hold_position_[0] = current_position_[0];
                hold_position_[1] = current_position_[1];
                hold_position_[2] = selfie_height_;
                hold_yaw_ = current_attitude_[2];
         }
        }
    } 
	double GetDistToGoal()
	{
		return sqrt(pow(goal_[0] - current_position_[0] , 2)
			+ pow(goal_[1] - current_position_[1] , 2)
			+ pow(goal_[2] - current_position_[2] , 2));
	}
	// add xy dist function
	double GetXYDistToGoal()
	{
		return sqrt(pow(goal_[0] - current_position_[0] , 2)
			+ pow(goal_[1] - current_position_[1] , 2));
	}
    void PrintGoal()
    {
        std::cout<<"Current Goal     : "<<goal_[0]<<", "<<goal_[1]<<", "<<goal_[2]<<std::endl;
    }
    //void PrintVelocity()
    //{
     //   std::cout<<"Current velocity : "<<goal_[0]<<", "<<goal_[1]<<", "<<goal_[2]<<std::endl;
    //}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PIDController");
	ros::NodeHandle nh;
  	ros::NodeHandle nh_param("~");
	PIDController controller(nh, nh_param);

	ros::Rate loop_rate(30);

	cout<<"start sub_Goal_PID controller"<<endl;


	// main loop
	static int count = 0;
	while(ok())
	{
		controller.PublishVelocity();

		loop_rate.sleep();
		spinOnce();

		if(count%10==0)
		{
			cout<<"==================="<<endl;
			cout<<"sub goal"<<std::endl;
			cout<<"Mode : "<<controller.GetMode()<<endl;
			cout<<"print count : "<<count<<endl;
            controller.PrintGoal();
		}
		count++;
	}
	return 0;
}

