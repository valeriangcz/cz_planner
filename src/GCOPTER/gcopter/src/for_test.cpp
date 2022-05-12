#include<iostream>
#include<string>
#include<Eigen/Eigen>
#include"gcopter/lbfgs.hpp"
#include"gcopter/sfc_gen.hpp"
#include"gcopter/test_gcopter.hpp"
#include<plan_env/grid_map.h>
#include"gcopter/trajectory.hpp"
#include"gcopter/dynamic_a_star.hpp"
using namespace std;
void eigen_map(){
    Eigen::VectorXd x(3+4);
    x<<1,2,3,4,5,6,7;
    std::cout<<"VectorXd x ="<<x.transpose()<<std::endl;
    Eigen::Map<Eigen::VectorXd> y(x.data(),3);
    y.resize(2);
    y[1] = 100;
    cout <<x.transpose()<<endl;
}

void eigne_normal(){
    Eigen::Matrix3d x;
    Eigen::MatrixXd y, z;
    x << 1,2,3,4,5,6,7,8,9;
    // Eigen::VectorXd vec;
    // vec.resize(4);
    // vec<<1,2,3,4;
    // cout << vec.head(0)<<endl;
    // cout << vec.head(4) <<endl;
    cout << x <<endl;
    
    cout << x.block<1,1>(0,0) <<endl;
    cout << x.block(1,1,1,1) <<endl;

}
void test_param(ros::NodeHandle& n){
    std::vector<double> vec;
    std::string str_1;
    if(n.getParam("vec",vec)) std::cout<<"vec get param successfully\n";
    else std::cout <<"vec failed to get param\n";

    if(n.getParam("vec",str_1)) std::cout <<"str get param successfully\n" << str_1 <<std::endl;
    else std::cout<<"str failed to get param\n";
}

class L_BFGS{
public:
    L_BFGS(int id):id(id){}
    lbfgs::lbfgs_parameter_t lbfgs_param;    
    int id;
    static inline double costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g){
        double cost;
        double x1 = x[0], x2 = x[1];
        cost = x1*x1 + x2*x2 +x1*x2 +2;
        g[0] = 2*x1 + x2;
        g[1] = x1 + 2*x2;
        return cost;
    }

    static inline int progress(void * ptr, const Eigen::VectorXd &x,
                               const Eigen::VectorXd &g,
                               const double fx,
                               const double step,
                               const int k,
                               const int ls){
                                //    L_BFGS &l = *(L_BFGS*) ptr;
                                   
                                   return 0;
    }

    inline double optimize(){
        Eigen::VectorXd x(2);
        x << 1, -4;
        // Eigen::VectorXd g(2);
        lbfgs_param.mem_size = 6;
        lbfgs_param.past = 5;
        lbfgs_param.g_epsilon = 0;
        double minCostFunctional;

        lbfgs::lbfgs_optimize(x, minCostFunctional, &L_BFGS::costFunctional,nullptr,progress,this,lbfgs_param);
        cout << "optimal x=" <<x.transpose()<<endl;
        cout << "minCostFunctional="<<minCostFunctional<<endl;
        return minCostFunctional;

    }
};


class test_a_star
{
public:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber target_sub;
    std::vector<Eigen::Vector3d> startGoal;
    Visualizer vis;
    voxel_map::VoxelMap::Ptr voxelMap;
    AStar::Ptr a_star;
    bool map_init;
    void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr & msg)
    {
        if (map_init) return;
        const Eigen::Vector3i xyz({500,500,30});
        const Eigen::Vector3d offset({-25.0,-25.0,0.0});
        voxelMap.reset(new voxel_map::VoxelMap(xyz,offset,0.1));
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
            voxelMap->setOccupied(Eigen::Vector3d(fdata[cur+0],fdata[cur+1],fdata[cur+2]));
        }
        voxelMap->dilate(2);
        map_init = true;
        a_star->initGridMap(voxelMap,Eigen::Vector3i(500,500,300));
        ROS_WARN("map received, A star is ready");
    }
    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr & msg)
    {
        if(!map_init) return;
        if( startGoal.size() >= 2 ) startGoal.clear();
        const Eigen::Vector3d goal( msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        if( voxelMap->query(goal) == 0 )
        {
            vis.visualizeStartGoal(goal,0.3,startGoal.size());
            startGoal.emplace_back(goal);
            ROS_WARN("set a goal");
        }
        a_star_search();

    }
    test_a_star(ros::NodeHandle & n):nh(n),vis(n),map_init(false)
    {
        map_sub = n.subscribe("/global_cloud",10,&test_a_star::mapCallBack,this);
        target_sub = n.subscribe("/goal",10,&test_a_star::targetCallBack,this);
        a_star.reset( new AStar);
    }
    void a_star_search()
    {
        if(startGoal.size() != 2 ) return;
        if(a_star->AstarSearch(0.1,startGoal[0],startGoal[1]))
        {
            ROS_WARN("a star search");
            std::vector<Eigen::Vector3d> route;
            route = a_star->getPath();
            Trajectory<5> traj;
            vis.visualize(traj,route);
        }
    }
};



int main(int argc, char** argv){
    ros::init(argc, argv,"for_test");
    ros::NodeHandle nh("~");
    test_a_star a(nh);
    ros::spin();

}