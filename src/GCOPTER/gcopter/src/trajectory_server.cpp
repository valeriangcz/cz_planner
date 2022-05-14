#include<ros/ros.h>
#include<string>
#include<iostream>
#include"gcopter/traj.h"
#include"gcopter/trajectory.hpp"
#include"quadrotor_msgs/PositionCommand.h"
#include"misc/visualizer.hpp"
ros::Publisher pos_cmd_pub;
quadrotor_msgs::PositionCommand cmd;


double pos_gain[3] = {0,0,0};
double vel_gain[3] = {0,0,0};
double time_forward_;
double traj_duration;
ros::Time start_time_;
double last_yaw_, last_yaw_dot_;
int receive_traj_ = 0;
Trajectory<5> traj; 
std::vector<double> emergency_stopPos;
void traj_callBack(const gcopter::traj::ConstPtr & traj_msg)
{
    ROS_WARN("receive traj");
    if(traj_msg->emergency_stop == 1)
    {
        receive_traj_ =2;
        emergency_stopPos = traj_msg->stop_pos;
        return;
    }
    std::vector<double> times = traj_msg->pieceTime;
    std::vector<double> coeff = traj_msg->serialB;
    Eigen::Map<Eigen::VectorXd> T (times.data(),times.size());
    Eigen::Map<Eigen::MatrixX3d> C (coeff.data(),coeff.size()/3,3);
    // std::cout <<"--------------------\n";
    // std::cout << T.transpose() <<std::endl;
    // std::cout <<"--------------------\n";
    // std::cout << C <<std::endl;
    // std::cout <<"--------------------\n";
    traj.clear();
    traj.reserve(int(times.size()));
    for (int i =0; i < int(times.size()); ++i )
    {
        traj.emplace_back(T(i),
        C.block<6,3>(6*i,0).transpose().rowwise().reverse());
    }
    traj_duration = traj.getTotalDuration();
    start_time_ = traj_msg->startTime;
    receive_traj_ = 1;
}
std::pair<double,double> calculate_yaw(
    double t_cur,
    Eigen::Vector3d &pos,
    ros::Time &time_now,
    ros::Time &time_last
)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration ? traj.getPos(t_cur + time_forward_) - pos : traj.getPos(traj_duration) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
    if (yaw_temp - last_yaw_ > PI)
    {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
        {
        yaw = last_yaw_ - max_yaw_change;
        if (yaw < -PI)
            yaw += 2 * PI;

        yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else
        {
        yaw = yaw_temp;
        if (yaw - last_yaw_ > PI)
            yawdot = -YAW_DOT_MAX_PER_SEC;
        else
            yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
        {
        yaw = last_yaw_ + max_yaw_change;
        if (yaw > PI)
            yaw -= 2 * PI;

        yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
        yaw = yaw_temp;
        if (yaw - last_yaw_ < -PI)
            yawdot = YAW_DOT_MAX_PER_SEC;
        else
            yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else
    {
        if (yaw_temp - last_yaw_ < -max_yaw_change)
        {
        yaw = last_yaw_ - max_yaw_change;
        if (yaw < -PI)
            yaw += 2 * PI;

        yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change)
        {
        yaw = last_yaw_ + max_yaw_change;
        if (yaw > PI)
            yaw -= 2 * PI;

        yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
        yaw = yaw_temp;
        if (yaw - last_yaw_ > PI)
            yawdot = -YAW_DOT_MAX_PER_SEC;
        else if (yaw - last_yaw_ < -PI)
            yawdot = YAW_DOT_MAX_PER_SEC;
        else
            yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}


void cmdCallBack(const ros::TimerEvent &e)
{
    if( receive_traj_ == 0 ) return;
    
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();
    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
    std::pair<double,double>yaw_yawdot(0,0);
    static ros::Time time_last = ros::Time::now();
    if (receive_traj_ == 2 )
    {
        // ROS_WARN("emergency stop!!!");
        pos = Eigen::Map<Eigen::Vector3d>(emergency_stopPos.data(),3);
    }
    //每一次进入cmdCallBack的时间
    else if (t_cur < traj_duration && t_cur >= 0)
    {
        pos = traj.getPos(t_cur);
        // visualizer.visualizeSphere(pos,1);

        vel = traj.getVel(t_cur);
        acc = traj.getAcc(t_cur);

        yaw_yawdot = calculate_yaw(t_cur,pos,time_now,time_last);

    }
    else if (t_cur >= traj_duration)
    {
        pos = traj.getPos(traj_duration);
        vel.setZero();
        acc.setZero();

        yaw_yawdot.first = last_yaw_;
        yaw_yawdot.second = 0;
    }
    else
    {
        ROS_WARN("[traj_server]: invalid time");
    }
    time_last = time_now;
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = 0;

    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);

    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);

    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd.yaw;

    pos_cmd_pub.publish(cmd);

}


int main(int argc, char** argv)
{

    ros::init(argc,argv,"traj_server");
    ros::NodeHandle nh("~");
    // Visualizer visualizer(nh);

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
    std::cout <<"time forward = " <<time_forward_<<"-----------" <<std::endl;
    // std::cout <<"----------------time_forward="<<time_forward_ <<"----------\n";
    
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    ros::Duration(1.0).sleep();

    ROS_WARN("[Traj server]: ready");

    ros::spin();
}
