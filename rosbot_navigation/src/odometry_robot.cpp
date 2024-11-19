#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class OdometryRobot : public rclcpp::Node
{
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    size_t count_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber;

public:
    OdometryRobot(): Node("odom_pub"), count_(0)
    {
        init_variables();
        // publisher
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("raw_odom", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&OdometryRobot::timer_callback, this));
        // subscriber
        subscriber = this->create_subscription<geometry_msgs::msg::Vector3>("wheel_vels", 10, std::bind(&OdometryRobot::get_velocities, this, _1));
    }

private:
    std::chrono::_V2::system_clock::time_point current_time;
    std::chrono::_V2::system_clock::time_point last_time; 
    std::chrono::_V2::system_clock::time_point then;
    std::chrono::_V2::system_clock::time_point t_next;

    geometry_msgs::msg::TransformStamped odom_trans;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    double x;
	double y;
	double th;
    double d;

	double vx;
	double vy;
	double vth;

	double dt;

	double last_vel_right;
	double last_vel_left;
	double distance_left;
	double distance_right;

	double dx,dr;
	double x_final;
	double y_final;
	double theta_final;

    double left_vel;
    double right_vel;

    double right_vel_fltr;
    double left_vel_fltr;

    double base_width;

    void init_variables();
    void timer_callback();
    void get_velocities(const geometry_msgs::msg::Vector3 &msg);
};

void OdometryRobot::init_variables()
{
    x = 0.0;
    y = 0.0;

    vx 	= 0.1;
	vy 	= 0.1;
	vth	= 0.1;

    left_vel = 0;
    right_vel = 0;
    last_vel_left = left_vel;
	last_vel_right = right_vel;

	x_final = 0;
	y_final = 0;
	theta_final = 0;

    base_width = 0.2;

    left_vel_fltr = 0;
    right_vel_fltr=0;

    current_time = std::chrono::high_resolution_clock::now();
    then = std::chrono::high_resolution_clock::now();
    last_time = std::chrono::high_resolution_clock::now();
    t_next = std::chrono::high_resolution_clock::now() + 100ms;

}
void OdometryRobot::timer_callback()
{
    double elapsed;
    current_time = std::chrono::high_resolution_clock::now();

    if(current_time > t_next){
        // dt = (current_time - last_time).seconds();
        elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time).count(); 
        dt = (double) elapsed/1000000;   // get time in seconds

        double right_vel_fltr = (right_vel + last_vel_right)/2;
        double left_vel_fltr = (left_vel + last_vel_left) /2;

        d = (double) (right_vel_fltr + left_vel_fltr) * dt / 2;
        th = (double) (right_vel_fltr - left_vel_fltr) * dt / base_width;
        // RCLCPP_INFO(this->get_logger(), "var check  %f", th);

        dx = d / dt;
        dr = th/ dt;

        if(d != 0){
            x = cos(th)*d;
            y = -sin(th)*d;
            x_final = x_final + ( cos(theta_final)*x - sin(theta_final)*y ); 
            y_final = y_final + ( sin(theta_final)*x + sin(theta_final)*y );
        }

        if(th != 0){
            theta_final = theta_final+ th;
        }

        geometry_msgs::msg::Quaternion odom_quat;
        odom_quat.x = 0;
        odom_quat.y = 0;
        odom_quat.z = sin(theta_final/2);
        odom_quat.w = cos(theta_final/2);

        // geometry_msgs::msg::TransformStamped odom_trans;

        // odom_trans.header.stamp = now();
        // odom_trans.header.frame_id = "raw_odom";
        // odom_trans.child_frame_id = "base_link";

        // odom_trans.transform.translation.x = x_final;
        // odom_trans.transform.translation.y = y_final;
        // odom_trans.transform.translation.z = 0;
        // odom_trans.transform.rotation = odom_quat;

        // odom_broadcaster->sendTransform(odom_trans);

        nav_msgs::msg::Odometry odom;

        odom.header.stamp = now();
        odom.header.frame_id = "raw_odom";

        odom.pose.pose.position.x = x_final;
        odom.pose.pose.position.y = y_final;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = dx;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = dr;

        publisher_->publish(odom);

        last_time = current_time;
        last_vel_left = left_vel;
        last_vel_right = right_vel;
    }
}

void OdometryRobot::get_velocities(const geometry_msgs::msg::Vector3 &msg) 
{
    left_vel = msg.y;
    right_vel = msg.x;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryRobot>());
    rclcpp::shutdown();
    return 0;
}