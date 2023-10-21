/*
 * Controller.cpp
 *
 *  Created on: 30 апр. 2017 г.
 *      Author: aminin
 */

#include "controller.h"

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>

namespace simple_controller
{

template <class T>
T clip(T val, T max)
{
  if ( val > max )
    return max;
  if ( val < -max)
    return -max;
  return val;
}


void Controller::purePursuite()
{
  // 设置前视距离
  const double lookaheadDistance = 5.0;

  // 初始化最小差值为一个大数
  double min_difference = 1e10;

  // 计算搜索范围，以确定在路径上搜索多远的点
  int search_len = static_cast<int>(lookaheadDistance / traj_dl) + 20;
  
  // 初始化索引值
  std::size_t index = nearest_point_index;
  std::size_t lookahead_index;
  double real_lookaheadDistance;

  // 遍历搜索范围内的路径点
  for (int index = nearest_point_index; index < nearest_point_index + search_len; ++index)
  {
    // 计算路径上的实际索引值，考虑环路路径情况
    std::size_t real_index;
    if (index >= 0 && index < static_cast<int>(path.poses.size()))
    {
      real_index = static_cast<std::size_t>(index);
    }
    if (index < 0)
    {
      real_index = (static_cast<int>(path.poses.size()) + index);
    }
    if (index >= static_cast<int>(path.poses.size()))
    {
      real_index = static_cast<std::size_t>(index) - path.poses.size();
    }

    // 计算机器人到路径点的距离
    const auto& path_point = path.poses[real_index].pose.position;
    double dx = robot_x - path_point.x;
    double dy = robot_y - path_point.y;
    double distance = sqrt(dx * dx + dy * dy);
    
    // 计算差值以寻找最接近前视距离的路径点
    double difference = abs(distance - lookaheadDistance);
    if (difference < min_difference)
    {
      // 更新最小差值
      min_difference = difference;
      
      // 记录最接近前视距离的路径点的索引
      nearest_point_index = real_index;
    }

    // 判断路径是否闭环，如果是，停止运动
    auto it_end = path.poses.cend();
    auto it_begin = path.poses.cbegin();
    if ((pow(it_end->pose.position.x - it_begin->pose.position.x, 2) + pow(it_end->pose.position.y - it_begin->pose.position.y, 2)) > 4)
    {
      std_msgs::Float32 vel_cmd;
      vel_cmd.data = 0.0;
      vel_pub.publish(vel_cmd);

      lookahead_index = path.poses.size() - 1;
      return;
    }
  }

  // 计算目标曲率所需的前视路径点
  const geometry_msgs::Pose& lookahead_pose = path.poses[lookahead_index].pose;
  double dx_target = lookahead_pose.position.x - robot_x;
  double dy_target = lookahead_pose.position.y - robot_y;
  double x_xAxis = cos(robot_theta - M_PI_2);
  double y_xAxis = cos(robot_theta - M_PI_2);

  // 计算横向误差
  double e_y = (dx_target * x_xAxis + dy_target * y_xAxis);

  // 计算目标曲率
  double targetCurvature = -2 * e_y / pow(real_lookaheadDistance, 2.0);

  // 发送目标曲率作为驱动器命令
  std_msgs::Float32 msgs;
  msgs.data = clip<double>(targetCurvature, max_curvature);
  steer_pub.publish(msgs);
}

void Controller::update_robot_pose(double dt)
{
  ROS_DEBUG_STREAM("update_robot_pose "<<dt<<" v = "<<current_linear_velocity );
  robot_x += current_linear_velocity * dt * sin(robot_theta);
  robot_y += current_linear_velocity * dt * cos(robot_theta);
  robot_theta = angles::normalize_angle(robot_theta + current_angular_velocity * dt);
  robot_time += ros::Duration(dt);
}


void Controller::on_path(const nav_msgs::Path& path) {
  ROS_INFO_STREAM("Got path " << path.poses.size());
  this->path = path;
  nearest_point_index = 0;
}

// search a pose in the pat nearest to the robot, assume path may be cyclic
std::size_t Controller::get_nearest_path_pose_index(int start_index,
                                                    std::size_t search_len)
{
  double nearest_distance = 1e10;
  std::size_t index = start_index;
  std::size_t nearest_index;
  geometry_msgs::Pose nearest_pose;
  for (int index = start_index; index < start_index + static_cast<int>(search_len); ++index) {
    std::size_t real_index;
    if (index >= 0 && index < static_cast<int>(path.poses.size())) {
        real_index = static_cast<std::size_t>(index);
    }
    if (index < 0) {
        real_index = (static_cast<int>(path.poses.size()) + index);
    }
    if (index >= static_cast<int>(path.poses.size())) {
        real_index = static_cast<std::size_t>(index) - path.poses.size();
    }

    const auto& path_point = path.poses[real_index].pose.position;
    double dx = robot_x - path_point.x;
    double dy = robot_y - path_point.y;
    double distance_sqr = dx * dx + dy * dy;
    if (distance_sqr < nearest_distance) {
        nearest_distance = distance_sqr;
        nearest_index = real_index;
    }
  }
  return nearest_index;
}

// 定时器回调函数

void Controller::on_timer(const ros::TimerEvent& event)
{
  if (std::abs(current_linear_velocity) < 0.01) {
    return;
  }
  update_robot_pose((ros::Time::now() - robot_time).toSec() );

  nearest_point_index = get_nearest_path_pose_index(nearest_point_index - 10, 20);

  const auto& nearest_pose = path.poses[nearest_point_index].pose;
  const auto& nearest_pose_angle = tf::getYaw(nearest_pose.orientation);
  double dx = robot_x - nearest_pose.position.x;
  double dy = robot_y - nearest_pose.position.y;
  // error is negative difference by y axe in the axis of the nereset pose
  double error = -(-dx * sin(nearest_pose_angle) + dy * cos(nearest_pose_angle));
  double diff_err = error - last_error;
  last_error = error;
  if ( fabs(error) < max_antiwindup_error )
    error_integral += error;
  else
    error_integral = 0.0;

  //Desired angular velocity
  double angular_cmd =  p_factor * error
                      + d_factor * diff_err
                      + i_factor * error_integral;
  //curvature for calculated angular velocity and for current linear velocity
  //计算角速度和当前线速度曲率
  double curvature = angular_cmd / current_linear_velocity;

  //send curvature as command to drives
  std_msgs::Float32 cmd;
  cmd.data = clip<double>(curvature, max_curvature);
  steer_pub.publish(cmd);

  //send trajectory for velocity controller
  publish_trajectory();

  //send error for debug proposes
  publish_error(error);
  ROS_DEBUG_STREAM("steering cmd = "<<curvature);
}

void Controller::on_pose(const nav_msgs::OdometryConstPtr& odom)
{
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_theta = 2*atan2(odom->pose.pose.orientation.z,
                        odom->pose.pose.orientation.w);

  world_frame_id = odom->header.frame_id;
  robot_time = odom->header.stamp;
}

void Controller::on_odo(const nav_msgs::OdometryConstPtr& odom)
{
  current_linear_velocity = odom->twist.twist.linear.x;
  current_angular_velocity = odom->twist.twist.angular.z;
  //ROS_DEBUG_STREAM("odom vel = "<<current_velocity);
}

void Controller::publish_error(double error)
{
  std_msgs::Float32 err_msg;
  err_msg.data = error;
  err_pub.publish(err_msg);
}

double Controller::cross_track_error()
{
  double error = 0.0;
  if (robot_y < radius)
  {
    double rx = robot_x;
    double ry = robot_y - radius;
    error = sqrt(rx*rx + ry*ry) - radius;
  }
  else if ( robot_y > cy)
  {
    double rx = robot_x;
    double ry = robot_y - cy;
    error = sqrt(rx*rx + ry*ry) - radius;
  }
  else if ( robot_x > 0 )
  {
    error = robot_x - radius;
  }
  else if ( robot_x < 0 )
  {
    error = -radius - robot_x;
  }
  return error;
}

void Controller::publish_trajectory()
{
 //  ROS_DEBUG_STREAM("publish trajectory");
  path_pub.publish(path);
}

void Controller::reset()
{
  error_integral = 0.0;
  last_error = cross_track_error();
}

void Controller::reset(double p, double d, double i)
{
  reset();
  p_factor = p;
  d_factor = d;
  i_factor = i;
}

nav_msgs::Path Controller::create_path() const { //轨迹被重新计算到 [nav_msgs::Path]
  //prepare path message from trajectory
  nav_msgs::Path path;
  path.header.frame_id = "odom";
  path.header.stamp = robot_time;
  auto segment_it = trajectory.begin();
  double previous_segment_left = 0.0;
  std::size_t points_added = 0;
  double point_length = 0.0;

  while (segment_it != trajectory.end()) {
    const auto segment = *segment_it;
    double segment_length = segment->get_length();
    //add  points from the segment
    while (point_length <= segment_length) {
      const auto point = segment->get_point(point_length);
      const auto angle = segment->get_orientation(point_length);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "odom";
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
      path.poses.push_back(pose);
      point_length += traj_dl;
      points_added++;
    }
    point_length -= segment_length;
    ++segment_it;
  }
  return path;
}

/*!
 * \brief constructor
 * loads parameters from ns
 * proportional, differential , integral - pid factors
 * max_antiwindup_error - max error for using integral component
 * trajectory consists of two circle segments connected with two lines
 * first circle center is (0, radius), second circle center is (0, cy)
 * radius - radius of circular parts
 * cy - center of second circle
 * traj_dl - discrete of published trajectory
 * traj_length - length of published trajectory
 * timer_period - timer discrete
 */
Controller::Controller(const std::string& ns):
    nh("~/" + ns),
    p_factor(nh.param("proportional", 1.0)),
    d_factor(nh.param("differential", 0.0)),
    i_factor(nh.param("integral", 0.0)),
    max_antiwindup_error( nh.param("max_antiwindup_error", 0.5) ),
    error_integral(0.0),
    last_error(0.0),
    radius(nh.param("radius", 10.0)),
    cy(nh.param("cy", 2*radius)), //default circle is in center (0,radius)
    max_curvature(nh.param("max_curvature", 0.2 )),
    traj_dl(nh.param("traj_dl", 0.2)),
    traj_length(nh.param("traj_length", 5.0)),
    pose_sub(nh.subscribe("ground_truth", 1, &Controller::on_pose, this)),
    odo_sub(nh.subscribe("odom", 1, &Controller::on_odo, this)),
    path_sub(nh.subscribe("path", 1, &Controller::on_path, this)),
    timer( nh.createTimer( ros::Duration(nh.param("timer_period", 0.1)), &Controller::on_timer, this ) ),
    err_pub(nh.advertise<std_msgs::Float32>("error", 10) ),
    steer_pub(nh.advertise<std_msgs::Float32>("/steering", 10)),
    path_pub(nh.advertise<nav_msgs::Path>("controller_path", 1))
{
  //counter clock
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
  trajectory.emplace_back( std::make_shared<trajectory::LinearSegment>  (        radius, radius, 0.0,   1.0,  cy - radius) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,   radius,   cy,   0.0,   1.0, M_PI/2*radius ) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,   0, radius + cy,   -1.0, 0.0, M_PI/2*radius ) );
  trajectory.emplace_back( std::make_shared<trajectory::LinearSegment>  (         -radius, cy,   0.0,   -1.0, cy - radius) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0/ radius,   -radius, radius, 0.0,  -1.0,  M_PI/2*radius) );

  //clock wise track
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (        radius, -radius, 0.0,   -1.0,  cy - radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   radius,   -cy,   0.0,   -1.0, M_PI/2*radius ) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   0, -radius - cy,   -1.0,  0.0, M_PI/2*radius ) );
//  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (       -radius, -cy,   0.0,   1.0, cy - radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0/ radius,   -radius, -radius, 0.0,  1.0,  M_PI/2*radius) );


  current_segment = trajectory.begin();
  const auto trajectory_path = create_path();
  on_path(trajectory_path);
}


Controller::~Controller()
{
  // TODO Auto-generated destructor stub
}

} /* namespace simple_controller */
