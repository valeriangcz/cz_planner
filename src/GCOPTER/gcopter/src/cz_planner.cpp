#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/traj.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include"gcopter/dynamic_a_star.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    std::string odomTopic;
    double dilateRadius;
    double voxelWidth;

    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    std::vector<double> chiVec;
    std::vector<double> vehicLastPos;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double timeoutRRT;
    double weightT;
    double smoothingEps;
    double relCostTol;
    int integralIntervs;

    double sensing_horizon;



    std::vector<double> mapBound;
    Config(){}
    Config(ros::NodeHandle& nh)
    {   
        //！！！！获取全局参数，需要加/！！！！！！！！！
        nh.getParam("/local_mapTopic",mapTopic);
        std::cout <<"-------------"<<mapTopic<<"------------\n";
        nh.getParam("dilateRadius",dilateRadius);
        std::cout <<"-------------"<<dilateRadius<<"------------\n";
        nh.getParam("VoxelWidth",voxelWidth);
        std::cout <<"-------------"<<voxelWidth<<"------------\n";
        nh.getParam("MapBound",mapBound);
        nh.getParam("vehicLastPos",vehicLastPos);
        nh.getParam("targetTopic",targetTopic);
        nh.getParam("/odomTopic",odomTopic);
        nh.getParam("TimeoutRRT", timeoutRRT);
        nh.getParam("MaxVelMag", maxVelMag);
        nh.getParam("MaxBdrMag", maxBdrMag);
        nh.getParam("MaxTiltAngle", maxTiltAngle);
        nh.getParam("MinThrust", minThrust);
        nh.getParam("MaxThrust", maxThrust);
        nh.getParam("VehicleMass", vehicleMass);
        nh.getParam("GravAcc", gravAcc);
        nh.getParam("HorizDrag", horizDrag);
        nh.getParam("VertDrag", vertDrag);
        nh.getParam("ParasDrag", parasDrag);
        nh.getParam("SpeedEps", speedEps);
        nh.getParam("WeightT", weightT);
        nh.getParam("ChiVec", chiVec);
        nh.getParam("SmoothingEps", smoothingEps);
        nh.getParam("IntegralIntervs", integralIntervs);
        nh.getParam("RelCostTol", relCostTol);
        nh.getParam("sensing_horizon",sensing_horizon);
    }
};


class CZ_planner
{
public:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber target_sub;
    ros::Subscriber odom_sub;
    ros::Publisher map_forVis_pub;
    ros::Publisher traj_pub;

    Eigen::Vector3d odom_vel;

    std::vector<Eigen::Vector3d> startGoal;
    voxel_map::VoxelMap local_map;
    voxel_map::VoxelMap::Ptr local_map_ptr;
    AStar::Ptr a_star;
    Config config;
    Visualizer visualizer;
    Trajectory<5> traj;


    ros::Publisher pos_pub;
    ros::Publisher vel_pub;
    ros::Publisher acc_pub;
    CZ_planner(ros::NodeHandle &n_):nh(n_),config(n_),visualizer(n_)
    {
        pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/pos",10);
        vel_pub = nh.advertise<geometry_msgs::PoseStamped>("/vel",10);
        acc_pub = nh.advertise<geometry_msgs::PoseStamped>("/acc",10);
        map_sub = n_.subscribe(config.mapTopic,10,&CZ_planner::mapCallback,this);
        // map_sub = n_.subscribe("/global_cloud",10,&CZ_planner::mapCallback,this);
        target_sub = n_.subscribe(config.targetTopic,2,&CZ_planner::tragetCallback,this);
        map_forVis_pub = n_.advertise<sensor_msgs::PointCloud2>("local_map",2);
        traj_pub = n_.advertise<gcopter::traj>("/traj",1);
        odom_sub = n_.subscribe(config.odomTopic, 10,&CZ_planner::odomCallback,this);
        //进行地图初始化，在pcl_render_node/cloud未发布，也就是还没有进入地图回调函数时，先进性初始化
        //在判断终点是否被占用时就会返回没有被占用
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);
        
        local_map = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
        local_map_ptr.reset(new voxel_map::VoxelMap(xyz,offset,config.voxelWidth));
        a_star.reset(new AStar);
        a_star->initGridMap(local_map_ptr,xyz);
    }

    inline void mapCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
    {

        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);
        
        local_map = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
        // ROS_WARN("enter mapCallback");
        size_t cur = 0;
        const size_t total = msg->data.size() / msg->point_step;
        float* fdata = (float*) (&msg->data[0]);

        for (size_t i =0 ; i < total; ++i )
        {   
            cur = msg->point_step / sizeof(float) *i ;
            if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
            {
                continue;
            }
            local_map.setOccupied(Eigen::Vector3d(fdata[cur+0],fdata[cur+1],fdata[cur+2]));

            local_map_ptr->setOccupied(Eigen::Vector3d(fdata[cur+0],fdata[cur+1],fdata[cur+2]));
        }
        local_map.dilate(std::ceil(config.dilateRadius/local_map.getScale()));

        local_map_ptr->dilate(std::ceil(config.dilateRadius/local_map.getScale()));
        a_star->initGridMap(local_map_ptr,xyz);
        ROS_WARN("a star map intialized");
        //将接收到的点云进行发布，用于可视化
        pcl::PointCloud<pcl::PointXYZ> local_cloud;
        pcl::fromROSMsg(*msg, local_cloud);
        sensor_msgs::PointCloud2 local_cloud_msg;
        pcl::toROSMsg(local_cloud, local_cloud_msg);
        local_cloud_msg.header.frame_id = "world";
        map_forVis_pub.publish(local_cloud_msg);
    }


    inline void tragetCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
    {
        if (startGoal.size() >= 2 )
        {
            startGoal.erase(startGoal.begin()+1,startGoal.end());
        }
        Eigen::Vector3d point (msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        if (local_map.query(point)==0)
        {
            startGoal.emplace_back(point);
            visualizer.visualizeStartGoal(point,0.4,startGoal.size());
        }
        else 
        {
            ROS_WARN("point is infeasible");
        }
        if (startGoal.size() == 2)
        {
            plan();
        }
    }
    inline void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        //首先保障startGoal的第一个坐标是无人机当前的odom
        Eigen::Vector3d cur_pos(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        odom_vel = Eigen::Vector3d(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
        if (startGoal.size() >= 1)
        {
            startGoal[0] = cur_pos;
        }
        else
        {
            startGoal.emplace_back(cur_pos);
        }
        //进行可视化
        // visualizer.visualizeSphere(cur_pos,0.3);
        //replan条件1，当前的odom与vehicLastPos距离超过无人机感知范围的2/3
        // Eigen::Vector3cd last_pos(config.vehicLastPos[0],config.vehicLastPos[1],config.vehicLastPos[2]);
        Eigen::Map<Eigen::Vector3d> last_pos(config.vehicLastPos.data(),3);
        if ( (cur_pos-last_pos).norm() > config.sensing_horizon*2.00 / 3.0 )
        {
            config.vehicLastPos = {cur_pos[0],cur_pos[1],cur_pos[2]};
            //进行重规划
            plan();
        }
        
        geometry_msgs::PoseStamped pos_msg;
        geometry_msgs::PoseStamped vel_msg;
        geometry_msgs::PoseStamped acc_msg;
        //计算traj的位置速度和加速度，进行发布
        if (traj.getPieceNum() > 0)
        {
            Eigen::Vector3d temp;
            double duration = ros::Time::now().toSec() - traj.startStamp;
            // std::cout <<"when replan duration is = "<< duration<<std::endl;
            // std::cout <<"total traj time is "<<traj.getTotalDuration() <<std::endl;
            if ( duration >=0 && duration < traj.getTotalDuration())
            {   
                temp = traj.getPos(duration);
                pos_msg.header.stamp = ros::Time::now();
                pos_msg.pose.position.x = temp[0];
                pos_msg.pose.position.y = temp[1];
                pos_msg.pose.position.z = temp[2];
                pos_pub.publish(pos_msg);
                temp = traj.getVel(duration);
                vel_msg.header.stamp = ros::Time::now();
                vel_msg.pose.position.x = temp[0];
                vel_msg.pose.position.y = temp[1];
                vel_msg.pose.position.z = temp[2];
                vel_pub.publish(vel_msg);
                temp = traj.getAcc(duration);
                acc_msg.header.stamp = ros::Time::now();
                acc_msg.pose.position.x = temp[0];
                acc_msg.pose.position.y = temp[1];
                acc_msg.pose.position.z = temp[2];
                acc_pub.publish(acc_msg);
            }
            
        }
        
        //replan条件2, 当前执行的轨迹与障碍物发生了碰撞,只检查2/3的轨迹
        if (traj.getPieceNum() > 0)
        {
            double time_step = 0.01;
            double t_cur = ros::Time::now().toSec() - traj.startStamp;
            double t_feature = t_cur;
            double time_emergency = 0.8;
            while ( t_feature > 0 && t_feature < 1.0*traj.getTotalDuration() / 3.0 )
            {
                Eigen::Vector3d feature_pos = traj.getPos(t_feature);
                if (local_map.query(feature_pos) == 1 )
                {
                    //发生碰撞
                    if( t_feature - t_cur < time_emergency )
                    {
                        //紧急停止，发布一条静止的轨迹
                        ROS_WARN("call emergency stop");
                        gcopter::traj traj_msg;
                        traj_msg.emergency_stop = 1;
                        traj_msg.stop_pos = std::vector<double>(cur_pos.data(),cur_pos.data()+cur_pos.size());
                        traj_pub.publish(traj_msg);
                    }
                    //进行replan
                    plan();
                    break;
                }
                t_feature += time_step;
            }
        }
       
        //

    }
    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            visualizer.visualizeSphere(startGoal[0],0.2);

            std::vector<Eigen::Vector3d> route;
            // sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
            //                                        startGoal[1],
            //                                        local_map.getOrigin(),
            //                                        local_map.getCorner(),
            //                                        &local_map, config.timeoutRRT,
            //                                        route);
            a_star->AstarSearch(0.1,startGoal[0],startGoal[1]);
            route = a_star->getPath();
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            std::cout <<"size of route =  "<<route.size()<<std::endl;
            local_map.getSurf(pc);
            sfc_gen::convexCover(route,
                                 pc,
                                 local_map.getOrigin(),
                                 local_map.getCorner(),
                                 7.0,
                                 1.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);

            if (route.size() > 1 )
            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                Eigen::Vector3d iniPos = route.front();
                Eigen::Vector3d iniVec = Eigen::Vector3d::Zero();
                Eigen::Vector3d iniAcc = Eigen::Vector3d::Zero();
                //起始状态需要根据当前执行的轨迹计算速度和加速度
                if (traj.getPieceNum() > 0)
                {
                    // ROS_WARN("cal init state");
                    double duration = ros::Time::now().toSec() - traj.startStamp;
                    if (duration >=0 && duration < traj.getTotalDuration() )
                    {
                        iniVec = traj.getVel(duration);
                        // std::cout <<"cal_vel="<<iniVec.transpose()<<std::endl;
                        iniAcc = traj.getAcc(duration); 
                        // std::cout <<"cal_acc="<<iniAcc.transpose()<<std::endl;
                    }
                }
            
                iniState << iniPos, iniVec, iniAcc;
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();
                // std::cout <<"4\n";
                //进行5次优化
                for (int i =0; i < 5; ++i )
                {
                    if ( gcopter.setup(config.weightT,iniState, finState,
                                   hPolys,INFINITY,config.smoothingEps,quadratureRes,
                                   magnitudeBounds,penaltyWeights,physicalParams))
                    {
                        if(!std::isinf(gcopter.optimize(traj,config.relCostTol)))
                        {
                            if (traj.getPieceNum() > 0)
                            {
                                //发布轨迹到traj_server
                                gcopter::traj traj_msg;
                                traj_msg.startTime = ros::Time::now();
                                traj_msg.order = 5;
                                traj_msg.pieceTime = std::vector<double>(
                                gcopter.traj_piece_time.data(),
                                gcopter.traj_piece_time.data()+gcopter.traj_piece_time.rows()*gcopter.traj_piece_time.cols()  
                                );
                                traj_msg.serialB = std::vector<double>(
                                    gcopter.traj_coeff_mat.data(),
                                    gcopter.traj_coeff_mat.data() + gcopter.traj_coeff_mat.cols()*gcopter.traj_coeff_mat.rows()
                                );
                                traj_pub.publish(traj_msg);
                                // ROS_WARN("publish traj");

                                visualizer.visualize(traj,route);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv,"cz_planner");
    ros::NodeHandle n("~");
    CZ_planner cz_planner(n);
    ros::spin();
    // ros::Rate rate(1);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }   
    return 0;
}