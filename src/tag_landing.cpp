
#include <tag_landing.hpp>


#define RadToDeg 180/M_PI

using namespace ros;
using namespace std;
void TagLandingController::VelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	vel_x_ = msg->twist.linear.x;	
	vel_y_ = msg->twist.linear.y;	
	vel_z_ = msg->twist.linear.z;	
}

void TagLandingController::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// if it detects something and currently hover, approach to the target!
	if(current_mode_ == MODE_HOVER)
	{	current_mode_ = MODE_FOLLOW;	}
	
	goal_[0] = msg->pose.position.x;
	goal_[1] = msg->pose.position.y;
	goal_[2] = landing_height_;

	// change mode to landing
	// 1. landing is ready
	// 2. height is below enough

	if(is_landing_ready_)
	{
		if(GetXYDistToGoal() < 0.15)
	 	{
	 		if(current_position_[2]<landing_height_+0.1)
	 		{
	 			//current_mode_ = MODE_LAND;
	 			//goal_[2] = 0;
	 		}
	 	}
	}
}

void TagLandingController::PublishVelocity()
{
	double KDx_ = 1/5 * Kx_;
	double KDy_ = 1/5 * Ky_;
	double KDz_ = 0;
	//calculate linear velocity
	double cmd_x = (goal_[0] - current_position_[0]) * Kx_ + (0-vel_x_) * KDx_;
	double cmd_y = (goal_[1] - current_position_[1]) * Ky_ + (0-vel_y_) * KDy_;
	double cmd_z = (goal_[2] - current_position_[2]) * Kz_ + (0-vel_z_) * KDz_;

	RegulateVelocity(cmd_x,velx_limit_);
	RegulateVelocity(cmd_y,vely_limit_);
	RegulateVelocity(cmd_z,velz_limit_);

	// current yaw angle
	double current_yaw = current_attitude_[2];
	
	double goal_yaw = 0;
	if(use_yaw_ == true )
	{	
		// if xy distance is more than 0.5m, head to goal
		if(GetXYDistToGoal() > 0.5)
		{
			goal_yaw = std::atan2(goal_[1]-current_position_[1],goal_[0]-current_position_[0]);	
			last_yaw_ = goal_yaw;
		}
		// Maintain last yaw
		else 
		{
			goal_yaw = last_yaw_;
		}
	}
	
	double yaw_diff = goal_yaw - current_yaw;

	// regulate yaw angle
	RegulateAngle(yaw_diff);
	
	double cmd_r = yaw_diff * Kyaw_;
	RegulateVelocity(cmd_r,yaw_limit_);
	
	geometry_msgs::TwistStamped command_msg;
	command_msg.header.stamp = ros::Time::now();

	// assign velocity
	command_msg.twist.linear.x = cmd_x;
	command_msg.twist.linear.y = cmd_y;
	command_msg.twist.linear.z = cmd_z; 
	command_msg.twist.angular.z = cmd_r; 
	pub_.publish(command_msg);
	//last_cmd_x_ = cmd_x;
	//last_cmd_y_ = cmd_y;

	// goal msg
	// publish goal message
	geometry_msgs::PoseStamped goal_msg;
	goal_msg.header.stamp = ros::Time::now();
	goal_msg.header.frame_id = "map";

	goal_msg.pose.position.x = goal_[0];
	goal_msg.pose.position.y = goal_[1];
	goal_msg.pose.position.z = goal_[2]; 
	
	pub_goal_.publish(goal_msg);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "TagLandingController");
	ros::NodeHandle nh;
  	ros::NodeHandle nh_param("~");
	TagLandingController controller(nh, nh_param);

	ros::Rate loop_rate(30);


	cout<<"start TagLanding controller"<<endl;

	// main loop
	static int count = 0;
	while(ok())
	{
		// controller.PublishVelocity();

		loop_rate.sleep();
		spinOnce();

		if(count%10==0)
		{
			cout<<"==================="<<endl;
			cout<<"print count : "<<count<<endl;
			controller.PrintInfo();
			count = 0;
		}
		count++;
	}
	return 0;
}

