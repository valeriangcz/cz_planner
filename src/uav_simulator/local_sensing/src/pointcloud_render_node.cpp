#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>
#include"visualization_msgs/MarkerArray.h"
using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud;
ros::Publisher pub_dynamic_cloud;
ros::Publisher pub_dynamic_preCloud;
sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;


ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;
ros::Timer local_sensing_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;

ros::Subscriber dynamic_obj_sub;
int dynamic_obj_num = 0;


inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  // ROS_WARN("odom received");
  has_odom = true;
  _odom = odom;
}
pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map, dynamic_obj_cloud, dynamic_objPre_cloud;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
pcl::search::KdTree<pcl::PointXYZ> _dynamicObjMap;
pcl::search::KdTree<pcl::PointXYZ> _dynamicObjPreMap;

vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map || !has_odom) return;
  bool has_point = false;
  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    
    has_point = true;
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      // if ((fabs(pt.z - _odom.pose.pose.position.z) / (pt.x - _odom.pose.pose.position.x)) >
      //     tan(M_PI / 12.0))
      //   continue;
      if ((fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon) >tan(M_PI / 6.0)) continue; 
      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);

      if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue; 
      _local_map.points.push_back(pt);
    }
  }
  if(dynamic_obj_num > 0 )
  { 
    _pointIdxRadiusSearch.clear();
    if(_dynamicObjMap.radiusSearch(searchPoint, sensing_horizon,
                                    _pointIdxRadiusSearch,
                                    _pointRadiusSquaredDistance) > 0){
    has_point = true;
    for (size_t i =0; i < _pointIdxRadiusSearch.size(); ++i){
      pt = dynamic_obj_cloud.points[_pointIdxRadiusSearch[i]];
      if ( (fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon ) > tan(M_PI /6.0)) continue;
      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x, 
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);
      if (pt_vec.normalized().dot(yaw_vec) < 0.5 ) continue;
      _local_map.points.push_back(pt);
      }
    }
    _pointIdxRadiusSearch.clear();
    if(_dynamicObjPreMap.radiusSearch(searchPoint, sensing_horizon, 
                                      _pointIdxRadiusSearch, _pointRadiusSquaredDistance) > 0){
      has_point = true;
      for(size_t i =0; i < _pointIdxRadiusSearch.size(); ++i ){
        pt = dynamic_objPre_cloud.points[_pointIdxRadiusSearch[i]];
        if ( (fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon ) > tan(M_PI /6.0)) continue;
        Vector3d pt_vec(pt.x - _odom.pose.pose.position.x, 
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);
      if (pt_vec.normalized().dot(yaw_vec) < 0.5 ) continue;
      // _local_map.points.push_back(pt);
      }                          
    }
  }
 

  if(!has_point) return;
  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  // _local_map_pcd.header.frame_id = "map";
  _local_map_pcd.header.frame_id = "world";
  pub_cloud.publish(_local_map_pcd);
}

void rcvLocalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
}

void rcvDynamicObjCallBack(const visualization_msgs::MarkerArray::ConstPtr & msg)
{
  // ROS_WARN("dynamic obj callback");
  dynamic_obj_num = msg->markers.size();
  if (dynamic_obj_num <= 0 ) return;
  dynamic_obj_cloud.clear();
  dynamic_objPre_cloud.clear();
  Eigen::Vector3d pos, vel, scale;
  for(int i =0; i < dynamic_obj_num; ++i )
  {
    scale(0) = msg->markers[i].scale.x;
    scale(1) = msg->markers[i].scale.y;
    scale(2) = msg->markers[i].scale.z;

    pos(0) = msg->markers[i].pose.position.x;
    pos(1) = msg->markers[i].pose.position.y;
    pos(2) = msg->markers[i].pose.position.z;

    vel(0) = msg->markers[i].pose.orientation.x;
    vel(1) = msg->markers[i].pose.orientation.y;
    vel(2) = msg->markers[i].pose.orientation.z;

    
    //根据位置和尺寸信息构建点云
    pcl::PointXYZ pt_random;
    double x, y, z;
    double resolution = msg->markers[i].pose.orientation.w;
    x = floor(pos(0)/resolution)*resolution; + resolution/2.0;
    y = floor(pos(1)/resolution)*resolution; + resolution/2.0;
    z = floor(pos(2)/resolution)*resolution; + resolution/2.0;
    int x_num, y_num, z_num;
    x_num = floor((scale(0)/resolution));
    y_num = floor((scale(1)/resolution));
    z_num = floor((scale(2)/resolution));
    for( int i = -x_num/2; i < x_num/2; i++){
      for(int j = -y_num/2; j < y_num/2; j++){
        for(int k = -z_num/2; k < z_num/2; k++){
          pt_random.x = x + (i+0.5)*resolution;
          pt_random.y = y + (j+0.5)*resolution;
          pt_random.z = z + (k+0.5)*resolution;
          dynamic_obj_cloud.push_back(pt_random);
          //预测运动，构建0.5s和1s后的障碍物
          pt_random.x = x+ vel(0)*0.5 + (i+0.5)*resolution;
          pt_random.y = y + vel(1)*0.5 +(j+0.5)*resolution;
          dynamic_objPre_cloud.push_back(pt_random);
          pt_random.x = x+  vel(0)*1 + (i+0.5)*resolution;
          pt_random.y = y + vel(1)*1 + (j+0.5)*resolution;
          dynamic_objPre_cloud.push_back(pt_random);
        }
      }
    }
  }
  if(dynamic_obj_cloud.size() <= 0 ) return;
  _dynamicObjMap.setInputCloud(dynamic_obj_cloud.makeShared());
  _dynamicObjPreMap.setInputCloud(dynamic_objPre_cloud.makeShared());
  
  sensor_msgs::PointCloud2 dynamic_cloud_msg;
  sensor_msgs::PointCloud2 dynamic_preCloud_msg;
  
  pcl::toROSMsg(dynamic_obj_cloud,dynamic_cloud_msg);
  dynamic_cloud_msg.header.frame_id="world";
  pub_dynamic_cloud.publish(dynamic_cloud_msg);

  pcl::toROSMsg(dynamic_objPre_cloud, dynamic_preCloud_msg);
  dynamic_preCloud_msg.header.frame_id="world";
  pub_dynamic_preCloud.publish(dynamic_preCloud_msg);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);

  nh.getParam("map/x_size", _x_size);
  nh.getParam("map/y_size", _y_size);
  nh.getParam("map/z_size", _z_size);

  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  // local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);
  dynamic_obj_sub = nh.subscribe("/dynamic/objs",1,rcvDynamicObjCallBack);
  // publisher depth image and color image
  pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 10);
  pub_dynamic_cloud=
      nh.advertise<sensor_msgs::PointCloud2>("/pcl_render/dynamic_obj",2);
  double sensing_duration = 1.0 / sensing_rate * 2.5;
  pub_dynamic_preCloud=
      nh.advertise<sensor_msgs::PointCloud2>("pcl_render/dynamic_pre_obj",2);
  local_sensing_timer =
      nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
