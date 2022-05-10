#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>

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
    std::vector<Eigen::Vector3d> startGoal;
    voxel_map::VoxelMap local_map;
    Config config;
    Visualizer visualizer;
    Trajectory<5> traj;
    double TrajStamp;

    CZ_planner(ros::NodeHandle &n_):nh(n_),config(n_),visualizer(n_)
    {
        
        map_sub = n_.subscribe(config.mapTopic,10,&CZ_planner::mapCallback,this);
        // map_sub = n_.subscribe("/global_cloud",10,&CZ_planner::mapCallback,this);
        target_sub = n_.subscribe(config.targetTopic,2,&CZ_planner::tragetCallback,this);
        map_forVis_pub = n_.advertise<sensor_msgs::PointCloud2>("local_map",2);
        odom_sub = n_.subscribe(config.odomTopic, 10,&CZ_planner::odomCallback,this);
        //进行地图初始化，在pcl_render_node/cloud未发布，也就是还没有进入地图回调函数时，先进性初始化
        //在判断终点是否被占用时就会返回没有被占用
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);
        
        local_map = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
        
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
        }
        local_map.dilate(std::ceil(config.dilateRadius/local_map.getScale()));
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
            for ( int i =0; i < 5; ++i )
            {
                if (plan()) 
                {
                    //将轨迹信息发布出去，参考ego-planner 中的traj_server需要哪些信息
                    //自定义traj消息类型
                    //编写traj
                    break;
                }
            }
        }
    }
    inline void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        //首先保障startGoal的第一个坐标是无人机当前的odom
        Eigen::Vector3d cur_pos(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        if (startGoal.size() >= 1)
        {
            startGoal[0] = cur_pos;
        }
        else
        {
            startGoal.emplace_back(cur_pos);
        }
        //进行可视化
        visualizer.visualizeSphere(cur_pos,0.3);
        //replan条件1，当前的odom与vehicLastPos距离超过无人机感知范围的2/3
        // Eigen::Vector3cd last_pos(config.vehicLastPos[0],config.vehicLastPos[1],config.vehicLastPos[2]);
        Eigen::Map<Eigen::Vector3d> last_pos(config.vehicLastPos.data(),3);
        if ( (cur_pos-last_pos).norm() > config.sensing_horizon*2.0 / 3.0 )
        {
            config.vehicLastPos = {cur_pos[0],cur_pos[1],cur_pos[2]};
            //进行重规划
            for(int i=0 ; i < 5; ++i ) if(plan()) break;
        }
        //replan条件2, 当前执行的轨迹与障碍物发生了碰撞
        //碰撞检查，计算从当前起到未来一段时间内是否碰撞，参考ego_planner
    }
    inline bool plan()
    {
        if (startGoal.size() == 2)
        {

            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   local_map.getOrigin(),
                                                   local_map.getCorner(),
                                                   &local_map, config.timeoutRRT,
                                                   route);
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;

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
                Eigen::Vector3d iniVec = Eigen::Vector3d::Zero();
                Eigen::Vector3d iniAcc = Eigen::Vector3d::Zero();
                //起始状态需要根据当前执行的轨迹计算速度和加速度
                if (traj.getPieceNum() > 0)
                {
                    double duration = ros::Time::now().toSec() - TrajStamp;
                    iniVec = traj.getVel(duration);
                    iniAcc = traj.getAcc(duration); 
                }
                iniState << route.front(), iniVec, iniAcc;
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
                
                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return false;
                }
                // std::cout <<"5\n";
                
                if (std::isinf(gcopter.optimize(traj,config.relCostTol)))
                { 
                    ROS_WARN("traj optimize fialed");
                    return false;
                }

                else
                {
                    if (traj.getPieceNum() > 0)
                    {
                        TrajStamp = ros::Time::now().toSec();
                        //发布轨迹到traj_server

                        visualizer.visualize(traj,route);
                    }
                    return true;
                }
              
            }
        }
        return false;
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