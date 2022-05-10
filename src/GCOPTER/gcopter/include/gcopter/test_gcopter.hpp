#ifndef TEST_GCOPTER_HPP
#define TEST_GCOPTER_HPP
#include"gcopter/voxel_map.hpp"
#include"gcopter/sfc_gen.hpp"
#include"gcopter/gcopter.hpp"
#include"misc/visualizer.hpp"

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/PoseStamped.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
struct Config{
    //地图相关
    std::string mapTopic;
    std::string targetTopic;
    std::vector<double> mapBound;
    double voxelWidth;
    double dilateRadius;
    //飞机相关，惩罚函数相关参数
    double maxVel;//最大速度
    double maxBdr;//最大角速度
    double maxTilt;//pitch最大角度
    double maxThrust;//最大推力
    double minThrust;//最小推力，
    std::vector<double>chiVec;//question 参数含义
    double vehicleMass;//飞机质量
    //动力学相关
    double gravAcc;//重力加速度
    double horizDrag;//水平阻力系数？
    double vertDrag;//垂直阻力系数？
    double parasDrag;//question 参数含义
    double speedEps;//question 参数含义
    //数值积分梯形公式分段段数
    int integralIntervs;
    
    double weightT; //惩罚函数时间项系数
    double smoothingEps;

    double relCostTol;
    Config(){}
    Config(const ros::NodeHandle &nh){
        nh.getParam("mapTopic", mapTopic);
        nh.getParam("targetTopic", targetTopic);
        nh.getParam("mapBound", mapBound);
        nh.getParam("voxelWidth", voxelWidth);
        nh.getParam("dilateRadius",dilateRadius);
    }
};
class test_gcopter{
public:
    voxel_map::VoxelMap voxelMap;
    ros::Subscriber map_sub;
    ros::Subscriber target_sub;
    std::vector<Eigen::VectorXd> startGoal;
    Config config;
    Visualizer visualizer;

    Trajectory<5> traj;
    bool MapIntialized;

    test_gcopter(ros::NodeHandle& nh):config(nh),visualizer(nh),MapIntialized(false){
        //初始化地图
        const Eigen::Vector3i size_xyz( (config.mapBound[1]-config.mapBound[0])/config.voxelWidth,
                                    (config.mapBound[3]-config.mapBound[2])/config.voxelWidth,
                                    (config.mapBound[5]-config.mapBound[4])/config.voxelWidth );
        const Eigen::Vector3d origin(config.mapBound[0], config.mapBound[2],config.mapBound[4]);
        voxelMap = voxel_map::VoxelMap(size_xyz,origin,config.voxelWidth);
        map_sub = nh.subscribe(config.mapTopic,1,&test_gcopter::mapCallBack,this);
        target_sub = nh.subscribe(config.targetTopic,1,&test_gcopter::targetCallBack,this);
    }

    void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg){
        if (!MapIntialized){
            ROS_WARN("map intialized !");
            size_t cur = 0;
            const size_t total = msg->data.size()/msg->point_step;//计算点云数据总个数
            float *fdata = (float*)(&msg->data[0]);//使用指针指向点云数据
            for(size_t i =0; i < total; ++i)
            {
                cur= msg->point_step / sizeof(float)*i;//一个点包含xyz三个坐标
                if (std::isnan(fdata[cur+0]) || std::isinf(fdata[cur+0]) ||
                    std::isnan(fdata[cur+1]) || std::isinf(fdata[cur+1]) ||
                    std::isnan(fdata[cur+2]) || std::isinf(fdata[cur+2]))
                    {
                        continue;
                    }
                //只有三个坐标都有有效数据时，为障碍物栅格的位置
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur+0],fdata[cur+1],fdata[cur+2]));
            }
            voxelMap.dilate(std::ceil(config.dilateRadius/voxelMap.getScale()));
            MapIntialized = true;
        }
    }

    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg){
        if (MapIntialized){
            if(startGoal.size() >= 2){
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius+
                                 fabs(msg->pose.orientation.z)*
                                     (config.mapBound[5] - config.mapBound[4] - 2* config.dilateRadius);
            Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if(voxelMap.query(goal) == 0 )
            {   
                ROS_WARN("receive goal!");
                startGoal.emplace_back(goal);
                visualizer.visualizeStartGoal(goal,0.3,startGoal.size());
            }
            else 
            {
                ROS_WARN("Infeasible Positon Selected !!!\n");
            }
            plan();             
        }
    }

    void plan(){
        if (startGoal.size() == 2 ){
            std::vector<Eigen::Vector3d> route;
            //前端搜索，生成轨迹，存入route中
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],startGoal[1],
                                                   voxelMap.getOrigin(),//地图原点vetor3d
                                                   voxelMap.getCorner(),//地图边界点，原点加边界长度
                                                   &voxelMap, 0.01,//地图提供一个qurey()函数，传入坐标时检查是否碰撞
                                                   route);
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);//获取障碍物膨胀后表面的点
            std::cout<<"test_gcopter.hpp/route.size()="<<route.size()<<std::endl;
            //步长太短凸包会生成失败
            sfc_gen::convexCover(route,pc,voxelMap.getOrigin(),
                                 voxelMap.getCorner(),7,3,hPolys);//7代表生成凸包的步长
            sfc_gen::shortCut(hPolys);
            if(route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                
                gcopter::GCOPTER_PolytopeSFC gcopter;
                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVel;
                magnitudeBounds(1) = config.maxBdr;
                magnitudeBounds(2) = config.maxTilt;
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

                if(! gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys,INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }
                if(std::isinf(gcopter.optimize(traj,config.relCostTol)))
                {
                    ROS_WARN("traj optimization faile!!!");
                    return;
                }

                if(traj.getPieceNum() >0)
                {
                    
                }

            }   
        }
    }
        
};
#endif
