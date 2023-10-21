/*
 * stage_controller.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: andreyminin
 */

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <algorithm>

ros::Publisher test_pub;//发布test
double desired_velocity = 0;//期望速度
double max_velocity = 10;//最大速度
double acc = 1.0;//加速时间
double max_test_time = 5.0;//最长测试时间 5s
double acc_time = max_test_time / 2.;//加速时间是最长测试时间除以2
double dcc_time = acc_time;//减速时间
ros::Time start_test_time;//开始测试时间
bool started = false;

void on_odo(const nav_msgs::Odometry& odom)//里程计时间
{
  double current_velocity = odom.twist.twist.linear.x;

}

void on_timer(const ros::TimerEvent& event) {
  const auto t = ros::Time::now();//设t为现在时刻
  if (!started) {
    start_test_time = t;//把现在时刻作为开始测试时刻
    started = true;
  }

  auto test_time = (t - start_test_time).toSec();//测试之后的“现在时刻”减去开始测试的“现在时刻”，转换成秒，得到测试时间

  std_msgs::Float32 vcmd;
  if (test_time >= max_test_time) {//如果测试时间大于最大测试时间
    desired_velocity = 0;//制动，期望速度为0
  } else {
    if (test_time <= acc_time) {//如果测试时间在最大时间范围内，如果测试时间小于等于加速时间
      desired_velocity = std::min(max_velocity, test_time * acc);//期望的速度就是最大速度，时间乘以加速度
    } else {
      if (test_time >= dcc_time) {//如果测试时间在最大时间范围内，如果测试时间大于加速时间，如果测试时间大于等于减速时间（画速度图就懂了）
        desired_velocity = std::max(0.0, max_velocity - acc * (test_time - dcc_time));
      } else {
        desired_velocity = max_velocity;
      }
    }
  }
  ROS_INFO_STREAM_COND(desired_velocity > 0.01, "test_time = " << test_time << " test vel = " << desired_velocity);
  vcmd.data = desired_velocity;
  test_pub.publish(vcmd);//发布的消息是期望速度
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "velocity_test");
  ros::NodeHandle nh("~");
  max_test_time = nh.param("test_time", 10.0);
  acc = nh.param("acc", 1.0);
  max_velocity = nh.param("max_velocity", 7.0);

  if (max_velocity / acc > max_test_time / 2.0) {
    acc_time = max_test_time / 2.0;
    dcc_time = max_test_time / 2.0;
    max_velocity = acc * max_test_time / 2.0;
    ROS_WARN_STREAM("Not enough time to reach max velocity " << max_velocity);
  } else {
    acc_time = max_velocity / acc;
    dcc_time = max_test_time - acc_time;
  }
  
  auto odo_sub = nh.subscribe("odom", 1, on_odo);
  test_pub = nh.advertise<std_msgs::Float32>("velocity", 1);

  if (nh.param("/use_sim_time", false)) {
    while(ros::ok()) {
      ros::spinOnce();
    
      start_test_time = ros::Time::now();
      if (!start_test_time.isZero()) {
        break;
      } 
    }
  }
  auto timer = nh.createTimer(ros::Duration(0.1), on_timer);
  start_test_time = ros::Time::now();
  ros::spin();
  return 0;
}


