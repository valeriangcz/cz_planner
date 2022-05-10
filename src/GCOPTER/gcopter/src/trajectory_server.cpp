#include<ros/ros.h>
#include<string>
#include<iostream>
#include"gcopter/traj.h"
#include"gcopter/trajectory.hpp"
#include"quadrotor_msgs/PositionCommand.h"

ros::Publisher pos_cmd_pub;
quadrotor_msgs::PositionCommand cmd;

double pos_gain[3] = {0,0,0};
double vel_gain[3] = {0,0,0};
double time_forward_;
double last_yaw_, last_yaw_dot_;
bool receive_traj_ = false;


void traj_callBack(const gcopter::traj::ConstPtr & traj_msg)
{
    // Trajectory<5> traj; 
    // std::vector<double> times = traj_msg->pieceTime;
    // std::vector<double> coeff = traj_msg->serialB;
    // Eigen::VectorXd T(times.data(),times.data() + times.size());
    // Eigen::Map<Eigen::VectorXd> T1(times.begin(),times.begin() + times.size());
    std::cout <<"receive traj\n";

}

void cmdCallBack(const ros::TimerEvent &e)
{

}

// std::pair<double,double> calculate_yaw(
//     double t_cur,
//     Eigen::Vector3d &pos,
//     ros::Time &time_now,
//     ros::Time &time_last
// ){}
// {
//     constexpr double PI = 3.1415926;
//     constexpr double YAW_DOT_MAX_PER_SEC = PI;
//     // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
//     std::pair<double, double> yaw_yawdot(0, 0);
//     double yaw = 0;
//     double yawdot = 0;

//     Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
//     double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
//     double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
//     if (yaw_temp - last_yaw_ > PI)
//     {
//         if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
//         {
//         yaw = last_yaw_ - max_yaw_change;
//         if (yaw < -PI)
//             yaw += 2 * PI;

//         yawdot = -YAW_DOT_MAX_PER_SEC;
//         }
//         else
//         {
//         yaw = yaw_temp;
//         if (yaw - last_yaw_ > PI)
//             yawdot = -YAW_DOT_MAX_PER_SEC;
//         else
//             yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
//         }
//     }
//     else if (yaw_temp - last_yaw_ < -PI)
//     {
//         if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
//         {
//         yaw = last_yaw_ + max_yaw_change;
//         if (yaw > PI)
//             yaw -= 2 * PI;

//         yawdot = YAW_DOT_MAX_PER_SEC;
//         }
//         else
//         {
//         yaw = yaw_temp;
//         if (yaw - last_yaw_ < -PI)
//             yawdot = YAW_DOT_MAX_PER_SEC;
//         else
//             yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
//         }
//     }
//     else
//     {
//         if (yaw_temp - last_yaw_ < -max_yaw_change)
//         {
//         yaw = last_yaw_ - max_yaw_change;
//         if (yaw < -PI)
//             yaw += 2 * PI;

//         yawdot = -YAW_DOT_MAX_PER_SEC;
//         }
//         else if (yaw_temp - last_yaw_ > max_yaw_change)
//         {
//         yaw = last_yaw_ + max_yaw_change;
//         if (yaw > PI)
//             yaw -= 2 * PI;

//         yawdot = YAW_DOT_MAX_PER_SEC;
//         }
//         else
//         {
//         yaw = yaw_temp;
//         if (yaw - last_yaw_ > PI)
//             yawdot = -YAW_DOT_MAX_PER_SEC;
//         else if (yaw - last_yaw_ < -PI)
//             yawdot = YAW_DOT_MAX_PER_SEC;
//         else
//             yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
//         }
//     }

//     if (fabs(yaw - last_yaw_) <= max_yaw_change)
//         yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
//     yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
//     last_yaw_ = yaw;
//     last_yaw_dot_ = yawdot;

//     yaw_yawdot.first = yaw;
//     yaw_yawdot.second = yawdot;

//     return yaw_yawdot;
// }
int main(int argc, char** argv)
{
    ros::init(argc,argv,"traj_server");
    ros::NodeHandle nh("~");

    ros::Subscriber traj_sub = nh.subscribe("/traj",10,traj_callBack);
    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01),cmdCallBack);
    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",50);

    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];

    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];

    nh.param("time_forward", time_forward_, -1.0);
    std::cout <<"----------------time_forward="<<time_forward_ <<"----------\n";
    
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    ros::Duration(1.0).sleep();

    ROS_WARN("[Traj server]: ready.");

    ros::spin();
}
