#include<iostream>
#include<string>
#include<Eigen/Eigen>
#include"gcopter/lbfgs.hpp"
#include"gcopter/sfc_gen.hpp"
#include"gcopter/test_gcopter.hpp"
#include<plan_env/grid_map.h>

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






int main(int argc, char** argv){
    // L_BFGS L_1(2);
    // L_1.optimize();

    // eigen_map();
    // eigne_normal();
    // cout << "hello\n";
    // std::cout<<"for test\n";
    // test_voxelMap(argc, argv);
    
    ros::init(argc, argv,"for_test");
    ros::NodeHandle nh("~");
    // test_gcopter gcoper(nh);
    // ros::Rate rate(10);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // GridMap gridmap;
    // gridmap.initMap(nh);

    // ros::spin();
//    Eigen::VectorXd points_1;
    Eigen::VectorXd v1(3);
    v1 << 1.2,3.4,5.6;
    Eigen::Matrix3d mat;
    mat<<v1,v1,v1;
    // std::vector<double> vec1 = std::vector<double>(v1.data(),v1.data()+v1.rows()*v1.cols());
    std::vector<double> vec1 = std::vector<double>(mat.data(),mat.data()+mat.rows()*mat.cols());
    // for(auto x: vec1) std::cout << x<<" ";
    Eigen::MatrixX3d mat1;
    mat1.resize(10,3);
    Eigen::MatrixXd mat2;
    mat2.resize(3,4);
    Eigen::VectorXd v2;
    v2.resize(100);
    // (3);
    std::cout << mat1.size() <<std::endl;
    std::cout << mat2.size() <<std::endl;
    std::cout << v2.size() <<std::endl;
    std::cout <<std::endl;

    

    return 0;
}