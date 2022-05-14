/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include"plan_env/linear_obj_model.hpp"
using namespace std;

int obj_num, _input_type;
double _x_size, _y_size, _h_size, _vel, _vel2, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1_x, _scale2_x, _scale1_y, _scale2_y, _scale1_z, _scale2_z, _interval, _resolution;

ros::Publisher obj_pub;            // visualize marker
ros::Publisher objs_pub; 
vector<ros::Publisher> pose_pubs;  // obj pose (from optitrack)
vector<ros::Publisher> vel_pubs;  // obj vel (from optitrack)
ros::Publisher dynamic_pcl_pub;    // point cloud of dynamic obstacles 
vector<LinearObjModel> obj_models;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_pos_x;
uniform_real_distribution<double> rand_pos_y;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_vel1;
uniform_real_distribution<double> rand_vel2;
uniform_real_distribution<double> rand_acc_r;
uniform_real_distribution<double> rand_acc_t;
uniform_real_distribution<double> rand_acc_z;
uniform_real_distribution<double> rand_color;
uniform_real_distribution<double> rand_scalex, rand_scaley, rand_scalez;
uniform_real_distribution<double> rand_yaw_dot;
uniform_real_distribution<double> rand_yaw;

ros::Time time_update, time_change;

void updateCallback(const ros::TimerEvent& e);
void visualizeObj(int id);
void visualizeObjAll();

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_obj");
  ros::NodeHandle node("~");
  /* ---------- initialize ---------- */
  node.param("obj_generator/obj_num", obj_num, 20);
  node.param("obj_generator/x_size", _x_size, 10.0);
  node.param("obj_generator/y_size", _y_size, 10.0);
  node.param("obj_generator/h_size", _h_size, 2.0);
  node.param("obj_generator/vel", _vel, 2.0);
  node.param("obj_generator/vel2", _vel2, 3.0);
  node.param("obj_generator/yaw_dot", _yaw_dot, 2.0);
  node.param("obj_generator/acc_r1", _acc_r1, 2.0);
  node.param("obj_generator/acc_r2", _acc_r2, 2.0);
  node.param("obj_generator/acc_z", _acc_z, 0.0);
  node.param("obj_generator/scale1x", _scale1_x, 0.5);
  node.param("obj_generator/scale2x", _scale2_x, 1.0);
  node.param("obj_generator/scale1y", _scale1_y, 0.5);
  node.param("obj_generator/scale2y", _scale2_y, 1.0);
  node.param("obj_generator/scale1z", _scale1_z, 0.5);
  node.param("obj_generator/scale2z", _scale2_z, 1.0);
  node.param("obj_generator/interval", _interval, 100.0);
  node.param("obj_generator/input_type", _input_type, 1);
  node.param("obj_generator/resolution", _resolution, 0.1);

  obj_pub = node.advertise<visualization_msgs::Marker>("/dynamic/obj", 10);
  objs_pub = node.advertise<visualization_msgs::MarkerArray>("/dynamic/objs", 10);

  for (int i = 0; i < obj_num; ++i) {
    ros::Publisher pose_pub =
        node.advertise<geometry_msgs::PoseStamped>("/dynamic/pose_" + to_string(i), 10);
    pose_pubs.push_back(pose_pub);

    ros::Publisher vel_pub =
        node.advertise<geometry_msgs::PoseStamped>("/dynamic/vel_" + to_string(i), 10);
    vel_pubs.push_back(vel_pub);
  }
  dynamic_pcl_pub = node.advertise<sensor_msgs::PointCloud2>("/dynamic/point_cloud", 10);

  ros::Timer update_timer = node.createTimer(ros::Duration(1 / 30.0), updateCallback);
  cout << "[dynamic]: initialize with " + to_string(obj_num) << " moving objs." << endl;
  ros::Duration(1.0).sleep();

  rand_color = uniform_real_distribution<double>(0.0, 1.0);
  rand_pos_x = uniform_real_distribution<double>(-_x_size/2, _x_size/2);
  rand_pos_y = uniform_real_distribution<double>(-_y_size/2, _y_size/2);
  rand_h = uniform_real_distribution<double>(_scale1_z, _scale2_z);

  rand_vel1 = uniform_real_distribution<double>(0.0, 1.0);
  int sign = rand_vel1(eng) > 0.5 ? -1 : 1;
  rand_vel2 = uniform_real_distribution<double>(_vel, _vel2);

  rand_acc_t = uniform_real_distribution<double>(0.0, 6.28);
  rand_acc_r = uniform_real_distribution<double>(_acc_r1, _acc_r2);
  rand_acc_z = uniform_real_distribution<double>(-_acc_z, _acc_z);


  rand_scalex = uniform_real_distribution<double>(_scale1_x, _scale2_x);
  rand_scaley = uniform_real_distribution<double>(_scale1_y, _scale2_y);
  rand_scalez = uniform_real_distribution<double>(_scale1_z, _scale2_z);

  rand_yaw = uniform_real_distribution<double>(0.0, 2 * 3.141592);
  rand_yaw_dot = uniform_real_distribution<double>(-_yaw_dot, _yaw_dot);

  /* ---------- give initial value of each obj ---------- */
  for (int i = 0; i < obj_num; ++i) {
    LinearObjModel model;
    Eigen::Vector3d pos(rand_pos_x(eng), rand_pos_y(eng), rand_h(eng));
    // Eigen::Vector3d pos(rand_pos_x(eng), rand_pos_y(eng),10);
    Eigen::Vector3d vel(sign * rand_vel2(eng), sign * rand_vel2(eng), 0.0);     // rand vel
    // Eigen::Vector3d vel(_vel, _vel, 0.0);                    // constant vel
    Eigen::Vector3d color(rand_color(eng), rand_color(eng), rand_color(eng));
    Eigen::Vector3d scale(rand_scalex(eng), rand_scaley(eng), rand_scalez(eng));
    double yaw = rand_yaw(eng);
    double yaw_dot = rand_yaw_dot(eng);

    double r, t, z;
    r = rand_acc_r(eng);
    t = rand_acc_t(eng);
    z = rand_acc_z(eng);
    Eigen::Vector3d acc(r * cos(t), r * sin(t), z);

    if ( _input_type == 1 )
    {
      model.initialize(pos, vel, acc, yaw, yaw_dot, color, scale, _input_type); // Vel input
    }
    else
    {
      model.initialize(pos, Eigen::Vector3d(0,0,0), acc, yaw, yaw_dot, color, scale, _input_type); // Acc input
    }
    model.setLimits(Eigen::Vector3d(_x_size/2, _y_size/2, _h_size), Eigen::Vector2d(0.0, _vel),
                    Eigen::Vector2d(0, 0));
    obj_models.push_back(model);
  }

  time_update = ros::Time::now();
  time_change = ros::Time::now();

  /* ---------- start loop ---------- */
  ros::spin();

  return 0;
}

void updateCallback(const ros::TimerEvent& e) {
  ros::Time time_now = ros::Time::now();

  /* ---------- update obj state ---------- */
  double dt = (time_now - time_update).toSec();
  time_update = time_now;
  for (int i = 0; i < obj_num; ++i) {
    obj_models[i].update(dt);
    visualizeObj(i);
    ros::Duration(0.000001).sleep();
  }
  visualizeObjAll();

  /* ---------- pub point cloud ---------- */
  pcl::PointCloud<pcl::PointXYZ> cloudMap;
  for (int i = 0; i < obj_num; ++i){
      Eigen::Vector3d dynamic_pos, scale;
      dynamic_pos = obj_models[i].getPosition();
      scale = obj_models[i].getScale();
      pcl::PointXYZ pt_random;

      double x,y,z;
      x = dynamic_pos(0); y = dynamic_pos(1); z = dynamic_pos(2);
      x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      y = floor(y / _resolution) * _resolution + _resolution / 2.0;
      z = floor(z / _resolution) * _resolution + _resolution / 2.0;
      
      int xnum, ynum, znum;
      xnum = ceil(scale(0) / _resolution);
      ynum = ceil(scale(1) / _resolution);
      znum = ceil(scale(2) / _resolution);

      for (int r = -xnum / 2; r < xnum / 2; r++)
        for (int s = -ynum / 2; s < ynum / 2; s++)
            for (int h = -znum / 2; h < znum / 2; h++){
                pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
                pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
                pt_random.z = z + (h + 0.5) * _resolution + 1e-2;
                cloudMap.points.push_back(pt_random);
            }
  }
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  sensor_msgs::PointCloud2 dynamic_pcd;
  pcl::toROSMsg(cloudMap, dynamic_pcd);
  dynamic_pcd.header.frame_id = "world";
  dynamic_pcl_pub.publish(dynamic_pcd);
}

void visualizeObj(int id) {
  Eigen::Vector3d pos, vel, color, scale;
  pos = obj_models[id].getPosition();
  vel = obj_models[id].getVelocity();
  color = obj_models[id].getColor();
  scale = obj_models[id].getScale();
  double yaw = obj_models[id].getYaw();

  Eigen::Matrix3d rot;
  rot << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

  Eigen::Quaterniond qua;
  qua = rot;

  /* ---------- rviz ---------- */
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = id;

  mk.scale.x = scale(0), mk.scale.y = scale(1), mk.scale.z = scale(2);
  mk.color.a = 0.7, mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2);

  // add orientation to the dynamic obstacles
  // mk.pose.orientation.w = qua.w();
  // mk.pose.orientation.x = qua.x();
  // mk.pose.orientation.y = qua.y();
  // mk.pose.orientation.z = qua.z();

  mk.pose.position.x = pos(0), mk.pose.position.y = pos(1), mk.pose.position.z = pos(2);

  obj_pub.publish(mk);

  /* ---------- pose ---------- */
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.seq = id;
  pose.pose.position.x = pos(0), pose.pose.position.y = pos(1), pose.pose.position.z = pos(2);
  pose.pose.orientation.w = 1.0;
  pose_pubs[id].publish(pose);

  /* ---------- vel ---------- */
  geometry_msgs::PoseStamped velocity;
  velocity.header.frame_id = "world";
  velocity.header.seq = id;
  velocity.pose.position.x = vel(0), velocity.pose.position.y = vel(1), velocity.pose.position.z = vel(2);
  velocity.pose.orientation.w = 1.0;
  vel_pubs[id].publish(velocity);
}

void visualizeObjAll() {
  visualization_msgs::Marker mk;
  visualization_msgs::MarkerArray mks;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;

  // add orientation to the dynamic obstacles
  // mk.pose.orientation.w = qua.w();
  // mk.pose.orientation.x = qua.x();
  // mk.pose.orientation.y = qua.y();
  // mk.pose.orientation.z = qua.z();


  for(int id = 0; id < obj_num; id++){

    Eigen::Vector3d pos, vel, color, scale;
    pos = obj_models[id].getPosition();
    vel = obj_models[id].getVelocity();
    color = obj_models[id].getColor();
    scale = obj_models[id].getScale();
    double yaw = obj_models[id].getYaw();

    mk.id = id;

    

    mk.scale.x = scale(0), mk.scale.y = scale(1), mk.scale.z = scale(2);

    mk.pose.position.x = pos(0), mk.pose.position.y = pos(1), mk.pose.position.z = pos(2); 

    mk.pose.orientation.w = _resolution;

    double vel_scale = 1.0, vel_scale2 = 1.0;

    mk.color.r = vel(0) * vel_scale, mk.color.g = vel(1) * vel_scale, mk.color.b = vel(2) * vel_scale; 

    // cout << vel.transpose() << endl;

    mk.pose.orientation.x = vel(0) * vel_scale2, mk.pose.orientation.y = vel(1) * vel_scale2, mk.pose.orientation.z = vel(2) * vel_scale2; 
    
    mk.color.a = 0.2;

    mks.markers.push_back(mk);

  }

  objs_pub.publish(mks);

}

