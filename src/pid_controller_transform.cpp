#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <array>

// 20230309
// This code will do transform when receives odometry
// from camera frame to world frame
// It is for vins!
using namespace ros;
using namespace std;
bool is_pose_available = false;


void RegulateVelocity(double& vel, const double limit)
{
	if(abs(vel) > limit)
	{
		vel = vel / abs(vel) * limit;
	}
}
class PIDController
{
private:
	NodeHandle nh_;
	Publisher pub_;
	Subscriber sub_;

	double current_position_[3];
	double goal_[3];

	double Kxy;
	double Kz;

public:
	// constructor
	PIDController() : goal_{0.0,0.0,0.0}
	{
		pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/scout/mavros/setpoint_velocity/cmd_vel",10);
		sub_ = nh_.subscribe("/scout/mavros/local_position/pose",1,&PIDController::OdomCallback, this);
		Kxy = 1.0;
		Kz  = 1.0;
	}
	void PublishVelocity()
	{
		double velxy_limit = 0.3;
		double velz_limit = 0.5;
		double cmd_x = (goal_[0] - current_position_[0]) * Kxy;
		double cmd_y = (goal_[1] - current_position_[1]) * Kxy;
		double cmd_z = (goal_[2] - current_position_[2]) * Kz;
		RegulateVelocity(cmd_x,velxy_limit);
		RegulateVelocity(cmd_y,velxy_limit);
		RegulateVelocity(cmd_z,velz_limit);
		
		geometry_msgs::TwistStamped command_msg;
		command_msg.header.stamp = ros::Time::now();

		command_msg.twist.linear.x = cmd_x;
		command_msg.twist.linear.y = cmd_y; 
		command_msg.twist.linear.z = cmd_z; 
		pub_.publish(command_msg);
	}
	void OdomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		current_position_[0] = msg->pose.position.x;	
		current_position_[1] = msg->pose.position.y;	
		current_position_[2] = msg->pose.position.z;	

		//PublishVelocity();
		is_pose_available = true;

	}
	void SetGoal(const std::array<double,3> msg)
	{
		goal_[0] = msg[0]; 
		goal_[1] = msg[1]; 
		goal_[2] = msg[2]; 
	}
	double GetDistToGoal()
	{
		return sqrt(pow(goal_[0] - current_position_[0] , 2)
			+ pow(goal_[1] - current_position_[1] , 2)
			+ pow(goal_[2] - current_position_[2] , 2));
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PIDController");
	PIDController controller;

	ros::Rate loop_rate(20);

	// define mission points
	std::vector<std::array<double,3>> GoalList;
	std::array<double,3> position_candidate;

	position_candidate[0] = 0.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = 0.0;
	GoalList.push_back(position_candidate);	

	position_candidate[0] = 0.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = 0.5;
	GoalList.push_back(position_candidate);	

	position_candidate[0] = 0.0;
	position_candidate[1] = 0.0;
	position_candidate[2] = 0.5;
	GoalList.push_back(position_candidate);	


	// main loop
	int path_length = GoalList.size();
	controller.SetGoal(GoalList.at(0));
	int index = 0;

	cout<<"start PID controller"<<endl;

	for(int i=0; i<GoalList.size(); i++)
	{
		cout<<"GOAL "<<i+1<<" : "<<GoalList.at(i)[0]<<", "<<GoalList.at(i)[1]<<", "<<GoalList.at(i)[2]<<", "<<endl;
	}

	static int count = 0;
	while(ok())
	{
		// check whether current goal is achieved.
		if (controller.GetDistToGoal() < 0.15 && index < (path_length-1))
		{
			// go to the next goal.
			index++;
			controller.SetGoal(GoalList.at(index));
		}
		controller.PublishVelocity();

		loop_rate.sleep();
		spinOnce();

		if(count>10)
		{
			cout<<"==================="<<endl;
			std::cout<<"Current index : "<<index<<endl
				<<"Last index : "<<path_length-1<<endl
				<<"GOAL : "<<GoalList.at(index)[0]<<", "<<GoalList.at(index)[1]<<", "<<GoalList.at(index)[2] <<endl;
			count = 0;
		}
		count++;
	}
	return 0;
}

