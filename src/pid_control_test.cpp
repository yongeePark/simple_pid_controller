#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

/**
 * 1. Subscribe the local position topic and save orientation into q
 * 2. Using q, convert it into roll, pitch, yaw
*/
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

class PIDController
{
public:
    PIDController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
        : nh_(nh)
        , nh_private_(nh_private)
    {
        cmd_pub_ = nh_.advertise<geometry
    }   
    ~PIDController(){} 
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    ros::Publisher  cmd_pub_;
    ros::Subscriber tag_pose_sub_;
    ros::Subscriber cur_pose_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_control_node");
    ros::NodeHandle nh, nh_private("~");
    ROS_INFO("[pid_control_test.cpp] Initialze node...");

    PIDController pid_node(nh, nh_private);
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        pid_node.publishVelocity();
        
        ROS_INFO("[pid_control_test.cpp] PID node is running...");
        loop_rate.sleep();
        spinOnce();
    }

    ROS_INFO("[pid_control_test.cpp] Terminate node...");
    return 0;
}