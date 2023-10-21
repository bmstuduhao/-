/*
 * Controller.h
 *
 *  Created on: 30 апр. 2017 г.
 *      Author: aminin
 */

#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <list>
#include <memory>

#include "trajectory_segment.h"

namespace simple_controller
{

using TrajPtr = std::shared_ptr<trajectory::TrajectorySegment>;

/*!
 *\brief robot controller 
 * \简要机器人控制器
 * controls following along defined trajectory via simple pid regulator
 * 通过简单的 PID 调节器控制沿着定义的轨迹行驶
 * angular_velocity = pid(error)
 * 角度速度 = pid(错误)
 * error is distance to trajectory
 * 误差是到轨迹的距离
 * trajectory is list of angular and linear segments, saved as pointers to base class Trajectory
 * 轨迹是角度和线性段的列表，保存为指向基类轨迹的指针
 * Trajectory is cycled
 * 轨迹循环
 * feedback from robot is received via ground_truth callback (real position of robot)
 * 通过ground_truth回调接收机器人的反馈（机器人的真实位置）
 * during control future trajectory is published for velocity controller
 * 在控制过程中，为速度控制器发布未来轨迹
 */

class Controller
{
protected:

  ros::NodeHandle nh;
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_theta = 0.0;
  //time of robot coordinates update 机器人坐标更新时间
  ros::Time robot_time;
  double p_factor;
  double d_factor;
  double i_factor;
  double max_antiwindup_error;
  double error_integral; //积分
  double last_error;

  ///\ circle params
  double radius;
  ///\ second circle center
  double  cy;

  double max_curvature; //max曲率

  double current_linear_velocity = 0.0;
  double current_angular_velocity = 0.0;
  //discrete of publish trajectory 发布轨迹的离散
  double traj_dl;
  //length of published trajectory 发布轨迹的长度
  double traj_length;

  using Trajectory = std::list<TrajPtr>; //std::list多个段组成的数据组
  /// \ container of trajectory segments
  std::list<TrajPtr> trajectory;

  nav_msgs::Path path;
  std::size_t nearest_point_index;

  /// \ current segment
  std::list<TrajPtr>::iterator current_segment;
  /// \ length of the current segment at the current point 当前点当前线段的长度
  double current_segment_length = 0.0;

  ros::Subscriber pose_sub;
  ros::Subscriber odo_sub;
  ros::Subscriber path_sub;
  ros::Timer timer;
  ros::Publisher err_pub; //发布当前控制error
  ros::Publisher steer_pub; //转动前轮
  ros::Publisher path_pub;
  ros::Publisher vel_pub;

  /// \ frame_id for coordinates of controller
  std::string world_frame_id;

  void purePursuite();
  void on_timer(const ros::TimerEvent& event); //定时器处理
  void on_pose(const nav_msgs::OdometryConstPtr& odom); //获取当前位置
  void on_path(const nav_msgs::Path& path);
  /*
   *@brief calculates feedback error for trajectory
   *@return feedback error
   */
  double cross_track_error();

  /// \ update robot pose to current time based on last pose and velocities
  void update_robot_pose(double dt);
  /*
   * \brief publishes trajectory as pointcloud message
   */
  void publish_trajectory(); //运动轨迹在rviz可视化
  void on_odo(const nav_msgs::OdometryConstPtr& odom); //使用里程计获取当前速度
  void publish_error(double error);
  nav_msgs::Path create_path() const;
  std::size_t get_nearest_path_pose_index(int start_index,
                                          std::size_t search_len);

public:
  double get_p_factor(){ return p_factor; }
  double get_d_factor(){ return d_factor; }
  double get_i_factor(){ return i_factor; }
  void reset();
  void reset(double p, double d, double i );
  Controller(const std::string& ns = "simple_controller");
  virtual ~Controller();
};

} /* namespace simple_controller */

#endif /* SRC_CONTROLLER_H_ */
