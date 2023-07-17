#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <vector>
#include <array>

#define RadToDeg 180/M_PI

using namespace ros;
using namespace std;

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
	Subscriber sub_local_goal_;

	double current_position_[3];
	double current_attitude_[3];
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

public:
	// constructor
	PIDController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param)
	:nh_(nh), nh_param_(nh_param), goal_{0.0,0.0,1.0}, use_yaw_(false), last_yaw(0.0), last_cmd_x_(0.0), last_cmd_y_(0.0)
	{
		pub_            = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);
		sub_            = nh_.subscribe("/mavros/local_position/pose",1,&PIDController::PoseCallback, this);
		sub_goal_       = nh_.subscribe("/photo_zone_pose",1,&PIDController::GoalCallback, this);
		sub_local_goal_ = nh_.subscribe("/apriltag_goal",1,&PIDController::LocalGoalCallback,this);

		// Parameter Setting
		if (!nh_param_.getParam("/sub_goal_pid/use_yaw",use_yaw_))
        {
            std::cout<<"[Warning] Please set [use_yaw_] parameter, default : false"<<std::endl;
            use_yaw_ = false;
        }
		if (!nh_param_.getParam("/sub_goal_pid/Kxy",Kx_))
        {
            std::cout<<"[Warning] Please set [Kx] parameter, default : 1.0"<<std::endl;
            Kx_ = 1.0;
        }
		if (!nh_param_.getParam("/sub_goal_pid/Ky",Ky_))
        {
            std::cout<<"[Warning] Please set [Ky] parameter, default : 1.0"<<std::endl;
            Ky_ = 1.0;
        }
		if (!nh_param_.getParam("/sub_goal_pid/Kz",Kz_))
        {
            std::cout<<"[Warning] Please set [Kz] parameter, default : 0.8"<<std::endl;
            Kz_ = 0.8;
        }
		if (!nh_param_.getParam("/sub_goal_pid/Kyaw",Kyaw_))
        {
            std::cout<<"[Warning] Please set [Kyaw] parameter, default : 3.0"<<std::endl;
            Kyaw_ = 3.0;
        }
		if (!nh_param_.getParam("/sub_goal_pid/velx_limit",velx_limit_))
        {
            std::cout<<"[Warning] Please set [velx_limit] parameter, default : 0.4"<<std::endl;
            velx_limit_ = 0.4;
        }
		if (!nh_param_.getParam("/sub_goal_pid/vely_limit",vely_limit_))
        {
            std::cout<<"[Warning] Please set [vely_limit] parameter, default : 0.4"<<std::endl;
            vely_limit_ = 0.4;
        }
		if (!nh_param_.getParam("/sub_goal_pid/velz_limit",velz_limit_))
        {
            std::cout<<"[Warning] Please set [velz_limit] parameter, default : 0.2"<<std::endl;
            velz_limit_ = 0.2;
        }
		if (!nh_param_.getParam("/sub_goal_pid/yaw_limit",yaw_limit_))
        {
            std::cout<<"[Warning] Please set [yaw_limit] parameter, default : 0.15"<<std::endl;
            yaw_limit_ = 0.15;
        }
		std::cout<<"================================="<<std::endl;
		std::cout<<"Parameters"<<std::endl
		<<"use_yaw_ : "<< (use_yaw_ ? "true" : "false")<<endl
		<<"Kxy_     : "<<Kxy_<<endl
		<<"Kz_      : "<<Kz_<<endl
		<<"Kyaw_    : "<<Kyaw_<<endl;
		std::cout<<"================================="<<std::endl;
	}
	void PublishVelocity()
	{
		//calculate linear velocity
		double cmd_x = (goal_[0] - current_position_[0]) * Kxy_;
		double cmd_y = (goal_[1] - current_position_[1]) * Kxy_;
		double cmd_z = (goal_[2] - current_position_[2]) * Kz_;
		
		// initial goal : 0,0,1
		
		if(GetXYDistToGoal() < 0.6)
		{
//			RegulateVelocity(cmd_x,0.15);
//			RegulateVelocity(cmd_y,0.15);
//			RegulateVelocity(cmd_z,0.15);

//			cmd_x = velxy_limit_;
//			RegulateVelocity(cmd_y,0.15); 
//			RegulateVelocity(cmd_z,0.15);

			cmd_x = last_cmd_x_;
			cmd_y = last_cmd_y_;
			RegulateVelocity(cmd_z,0.15);
		}		
		else
		{
			RegulateVelocity(cmd_x,velxy_limit_);
			RegulateVelocity(cmd_y,velxy_limit_);
			RegulateVelocity(cmd_z,velz_limit_);
		}
		


		// current yaw angle
		double current_yaw = current_attitude_[2];
		
		double goal_yaw = 0;
		if(use_yaw_ == true )
		{	
			if(GetXYDistToGoal() > 0.6)
			{
				goal_yaw = std::atan2(goal_[1]-current_position_[1],goal_[0]-current_position_[0]);	
				last_yaw_ = goal_yaw;
			}
			else // Use yaw, but drone is close enough to the target
			{
				goal_yaw = last_yaw_;
			}
		}
		
		
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

		if ( current_position_[2] < 0.3) // do not change yaw if height is lower than 0.3
		{   cmd_r = 0;	}

		// assign
		command_msg.twist.linear.x = cmd_x;
		command_msg.twist.linear.y = cmd_y; 
		command_msg.twist.linear.z = cmd_z; 

		
		//command_msg.twist.angular.z = 0; 
		command_msg.twist.angular.z = cmd_r; 
		pub_.publish(command_msg);
		last_cmd_x_ = cmd_x;
		last_cmd_y_ = cmd_y;
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
	void LocalGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
	{
		if(is_pose_available)
		{
			// Get local position x,y,z
			double local_x = msg->pose.position.x;
			double local_y = msg->pose.position.y;
			double local_z = msg->pose.position.z;

			// Rotate into global frame
			// yaw angle : current_attitude_[2]
			// cuurent x,y,z : current_position_ [0,1,2]
			double global_x = cos(current_attitude_[2]) * local_x - sin(current_attitude_[2]) * local_y + current_position_[0];
			double global_y = sin(current_attitude_[2]) * local_x + cos(current_attitude_[2]) * local_y + current_position_[1];
			double global_z = local_z + current_position_[2];
			
			goal_[0] = global_x;
			goal_[1] = global_y;
			goal_[2] = global_z;
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

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PIDController");
	ros::NodeHandle nh;
  	ros::NodeHandle nh_param("~");
	PIDController controller(nh, nh_param);

	ros::Rate loop_rate(30);


	/*
	// define mission points
	std::vector<std::array<double,3>> GoalList;
	std::array<double,3> position_candidate;

	// 1st
	position_candidate[0] = 0.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = 1.2;
	GoalList.push_back(position_candidate);	

	position_candidate[0] = 1.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = 1.2;
	GoalList.push_back(position_candidate);	

	
	position_candidate[0] = 6.0;
	position_candidate[1] = 3.0;
	position_candidate[2] = 1.2;
	GoalList.push_back(position_candidate);	

	
	position_candidate[0] = 9.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = 1.2;
	GoalList.push_back(position_candidate);	

	position_candidate[0] = 6.0;
	position_candidate[1] = -3.0;
	position_candidate[2] = 1.2;
	GoalList.push_back(position_candidate);	

	position_candidate[0] = 1.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = 1.2;
	GoalList.push_back(position_candidate);	

	position_candidate[0] = 0.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = 1.2;
	GoalList.push_back(position_candidate);	

	position_candidate[0] = 0.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = -0.1;
	GoalList.push_back(position_candidate);	
	*/


	/*
	int path_length = GoalList.size();
	controller.SetGoal(GoalList.at(0));
	int index = 0;
	*/

	cout<<"start sub_Goal_PID controller"<<endl;

	/*
	for(int i=0; i<GoalList.size(); i++)
	{
		cout<<"GOAL "<<i+1<<" : "<<GoalList.at(i)[0]<<", "<<GoalList.at(i)[1]<<", "<<GoalList.at(i)[2]<<", "<<endl;
	}
	*/

	// main loop
	static int count = 0;
	while(ok())
	{
		/*
		// check whether current goal is achieved.
		if (controller.GetDistToGoal() < 0.25 && index < (path_length-1))
		{
			// go to the next goal.
			index++;
			controller.SetGoal(GoalList.at(index));
		}
		*/
		controller.PublishVelocity();

		loop_rate.sleep();
		spinOnce();

		if(count%10==0)
		{
			cout<<"==================="<<endl;
			cout<<"print count : "<<count<<endl;
			cout<<"sub goal"<<std::endl;
			/*
			std::cout<<"Current index : "<<index<<endl
				<<"Last index : "<<path_length-1<<endl
				<<"GOAL : "<<GoalList.at(index)[0]<<", "<<GoalList.at(index)[1]<<", "<<GoalList.at(index)[2] <<endl;
			*/
		}
		count++;
	}
	return 0;
}

