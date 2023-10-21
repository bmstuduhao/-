
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <algorithm>

//global publishers
ros::Publisher throttle_pub;
ros::Publisher err_pub;
// gobal variables 全局变量
double desired_velocity = 0;//期望速度
double current_velocity = 0;//当前速度
ros::Time last_timer_time;//定义0速度时刻
// Controller gains
double Kp = 200.0;  // Proportional gain比例增益
double Ki = 30.0;  // Integral gain积分增益
double Kd = 3.0; // Derivative gain衍生收益
// Error variables for PID controlPID 控制的误差变量
double prev_error = 0;
double error_integral = 0;
double error_derivative = 0;
const double alpha = 0.2;
// Apply control limits for safety应用安全控制限值
double max_throttle = 200.0;  // Maximum allowed throttle最大允许油门
double max_velocity = 10.0;

void on_command_velocity(const std_msgs::Float32& msg) {
  desired_velocity = msg.data;
  std_msgs::Float32 err;
  err.data = desired_velocity - current_velocity;//速度差=期望速递-当前速度
  // ROS_INFO_STREAM_COND(cmd_velocity > 0.1, "current velocity " << current_velocity << " err = " << err.data);
  err_pub.publish(err);//速度差发布
}

void on_odo(const nav_msgs::Odometry& odom)
{
  double new_velocity = odom.twist.twist.linear.x;
  current_velocity = alpha * new_velocity + (1.0 - alpha) * current_velocity;
  
}


void on_timer(const ros::TimerEvent& event) {//timer是向throttle话题发布加速/制动消息的
  auto t = ros::Time::now();//现在时间
  auto dt = (t - last_timer_time).toSec();//现在时间减去速度为0的时刻，换算成秒，行驶的时间
  last_timer_time = t;//把现在的时间变为起始时间
  std_msgs::Float32 throttle_cmd;//发布的消息

  // place code here to calculate throttle/brake在这里放置代码来计算油门/刹车

  double error = desired_velocity - current_velocity;
  error_integral += error * dt;
  error_derivative = (error - prev_error) / dt;
  double control_command = Kp * error + Ki * error_integral + Kd * error_derivative;

  auto clip = [](double _d1, double _d2) {
    if (_d1 > _d2) return _d2;
    if (_d1 < -_d2) return -_d2;
    return _d1;
  };

  desired_velocity = clip(desired_velocity, max_velocity);
  throttle_cmd.data = clip(Kp * error + Ki * error_integral + Kd * error_derivative, max_throttle);
  // Publish the throttle/brake command发布油门/刹车指令
  throttle_pub.publish(throttle_cmd);

  // Update previous error for the next iteration为下一次迭代更新之前的错误
  prev_error = error;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "velocity_controller");//node name
  ros::NodeHandle nh("~");
  throttle_pub = nh.advertise<std_msgs::Float32>("throttle", 1);
  auto odo_sub = nh.subscribe("odom", 1, on_odo);//接收里程计消息
  err_pub = nh.advertise<std_msgs::Float32>("velocity_err", 1);
  auto cmd_sub = nh.subscribe("velocity", 1, on_command_velocity);//发布了速度差
  if (nh.param("/use_sim_time", false)) {
    while(ros::ok()) {
      ros::spinOnce();
    
      last_timer_time = ros::Time::now();
      if (!last_timer_time.isZero()) {
        break;
      } 
    }
  }
  // Initialize the timer and previous error初始化计时器和之前的错误
  auto timer = nh.createTimer(ros::Duration(0.1), on_timer);
  last_timer_time = ros::Time::now();
  prev_error = 0;
  error_integral = 0;
  ros::spin();
  return 0;
}


