#include <chrono>
#include <memory>
#include <mutex>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "twist_pid_controller/msg/pid_debug.hpp"

using namespace std::chrono_literals;

// Generic PID controller class
class PID {
public:
  PID(double kp = 0, double ki = 0, double kd = 0, double max_i = 0)
  : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_i) {}

  void setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setMaxIntegral(double max_i) {
    max_integral_ = max_i;
  }

  double compute(double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;

    // Reset integral if error changes sign or setpoint is very small
    if ((std::signbit(error) != std::signbit(prev_error_)) || std::abs(setpoint) < deadband_) {
      integral_ = 0.0;
    }

    // Integrate error
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -max_integral_, max_integral_);

    // Derivative
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    // PID output
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

private:
  double kp_, ki_, kd_;          // PID gains
  double max_integral_;          // maximum absolute integral term
  double deadband_{0.05};        // deadband for integral reset
  double integral_{0.0};         // accumulated integral
  double prev_error_{0.0};       // last error for derivative
};

class TwistPIDController : public rclcpp::Node {
public:
  TwistPIDController()
  : Node("twist_pid_controller")
  {
    loadParameters();

    setupCommunication();

    RCLCPP_INFO(get_logger(), "Twist PID Controller node initialized");
  }

private:
  // Member PIDs
  PID pid_lin_x_, pid_lin_y_, pid_lin_z_;
  PID pid_ang_x_, pid_ang_y_, pid_ang_z_;

  // Parameter storage
  double kp_lin_x_{0}, ki_lin_x_{0}, kd_lin_x_{0};
  double kp_lin_y_{0}, ki_lin_y_{0}, kd_lin_y_{0};
  double kp_lin_z_{0}, ki_lin_z_{0}, kd_lin_z_{0};
  double kp_ang_x_{0}, ki_ang_x_{0}, kd_ang_x_{0};
  double kp_ang_y_{0}, ki_ang_y_{0}, kd_ang_y_{0};
  double kp_ang_z_{0}, ki_ang_z_{0}, kd_ang_z_{0};

  bool feed_forward_{true};
  double max_int_lin_{1.0}, max_int_ang_{1.0};
  double control_frequency_{100.0};
  double timeout_{0.5};
  bool debug_{true};
  std::string debug_topic_{"pid_debug"};
  std::string joy_topic_{"joy"};
  double k_increment_{0.05};

  std::string cmd_vel_in_{"/nav_vel_cmd"};
  std::string feedback_topic_{"/local_odometry"};
  std::string cmd_vel_out_{"/nav_vel"};

  // State
  std::mutex mutex_;
  geometry_msgs::msg::TwistStamped desired_;  // last commanded twist
  geometry_msgs::msg::TwistStamped feedback_; // last measured twist
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_fb_time_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fb_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void loadParameters() {
    // Linear X gains
    this->declare_parameter("kp_linear_x", kp_lin_x_);
    this->declare_parameter("ki_linear_x", ki_lin_x_);
    this->declare_parameter("kd_linear_x", kd_lin_x_);
    this->get_parameter("kp_linear_x", kp_lin_x_);
    this->get_parameter("ki_linear_x", ki_lin_x_);
    this->get_parameter("kd_linear_x", kd_lin_x_);
    pid_lin_x_.setGains(kp_lin_x_, ki_lin_x_, kd_lin_x_);

    // Linear Y gains
    this->declare_parameter("kp_linear_y", kp_lin_y_);
    this->declare_parameter("ki_linear_y", ki_lin_y_);
    this->declare_parameter("kd_linear_y", kd_lin_y_);
    this->get_parameter("kp_linear_y", kp_lin_y_);
    this->get_parameter("ki_linear_y", ki_lin_y_);
    this->get_parameter("kd_linear_y", kd_lin_y_);
    pid_lin_y_.setGains(kp_lin_y_, ki_lin_y_, kd_lin_y_);

    // Linear Z gains
    this->declare_parameter("kp_linear_z", kp_lin_z_);
    this->declare_parameter("ki_linear_z", ki_lin_z_);
    this->declare_parameter("kd_linear_z", kd_lin_z_);
    this->get_parameter("kp_linear_z", kp_lin_z_);
    this->get_parameter("ki_linear_z", ki_lin_z_);
    this->get_parameter("kd_linear_z", kd_lin_z_);
    pid_lin_z_.setGains(kp_lin_z_, ki_lin_z_, kd_lin_z_);

    // Angular X gains
    this->declare_parameter("kp_angular_x", kp_ang_x_);
    this->declare_parameter("ki_angular_x", ki_ang_x_);
    this->declare_parameter("kd_angular_x", kd_ang_x_);
    this->get_parameter("kp_angular_x", kp_ang_x_);
    this->get_parameter("ki_angular_x", ki_ang_x_);
    this->get_parameter("kd_angular_x", kd_ang_x_);
    pid_ang_x_.setGains(kp_ang_x_, ki_ang_x_, kd_ang_x_);

    // Angular Y gains
    this->declare_parameter("kp_angular_y", kp_ang_y_);
    this->declare_parameter("ki_angular_y", ki_ang_y_);
    this->declare_parameter("kd_angular_y", kd_ang_y_);
    this->get_parameter("kp_angular_y", kp_ang_y_);
    this->get_parameter("ki_angular_y", ki_ang_y_);
    this->get_parameter("kd_angular_y", kd_ang_y_);
    pid_ang_y_.setGains(kp_ang_y_, ki_ang_y_, kd_ang_y_);

    // Angular Z gains
    this->declare_parameter("kp_angular_z", kp_ang_z_);
    this->declare_parameter("ki_angular_z", ki_ang_z_);
    this->declare_parameter("kd_angular_z", kd_ang_z_);
    this->get_parameter("kp_angular_z", kp_ang_z_);
    this->get_parameter("ki_angular_z", ki_ang_z_);
    this->get_parameter("kd_angular_z", kd_ang_z_);
    pid_ang_z_.setGains(kp_ang_z_, ki_ang_z_, kd_ang_z_);

    // Feed-forward
    this->declare_parameter("feed_forward", feed_forward_);
    this->get_parameter("feed_forward", feed_forward_);

    // Integrator limits
    this->declare_parameter("max_integral_linear", max_int_lin_);
    this->declare_parameter("max_integral_angular", max_int_ang_);
    this->get_parameter("max_integral_linear", max_int_lin_);
    this->get_parameter("max_integral_angular", max_int_ang_);
    pid_lin_x_.setMaxIntegral(max_int_lin_);
    pid_lin_y_.setMaxIntegral(max_int_lin_);
    pid_lin_z_.setMaxIntegral(max_int_lin_);
    pid_ang_x_.setMaxIntegral(max_int_ang_);
    pid_ang_y_.setMaxIntegral(max_int_ang_);
    pid_ang_z_.setMaxIntegral(max_int_ang_);

    // Timing
    this->declare_parameter("control_frequency", control_frequency_);
    this->declare_parameter("timeout", timeout_);
    this->get_parameter("control_frequency", control_frequency_);
    this->get_parameter("timeout", timeout_);

    // Debug
    this->declare_parameter("debug", debug_);
    this->declare_parameter("debug_topic", debug_topic_);
    this->declare_parameter("joy_topic", joy_topic_);
    this->declare_parameter("k_increment", k_increment_);
    this->get_parameter("debug", debug_);
    this->get_parameter("debug_topic", debug_topic_);
    this->get_parameter("joy_topic", joy_topic_);
    this->get_parameter("k_increment", k_increment_);

    // Topics
    this->declare_parameter("cmd_vel_input_topic", cmd_vel_in_);
    this->declare_parameter("feedback_vel_topic", feedback_topic_);
    this->declare_parameter("cmd_vel_out_topic", cmd_vel_out_);
    this->get_parameter("cmd_vel_input_topic", cmd_vel_in_);
    this->get_parameter("feedback_vel_topic", feedback_topic_);
    this->get_parameter("cmd_vel_out_topic", cmd_vel_out_);

    RCLCPP_INFO(this->get_logger(), "PID gains: lin_x: Kp=%.3f Ki=%.3f Kd=%.3f | ang_z: Kp=%.3f Ki=%.3f Kd=%.3f",
          kp_lin_x_, ki_lin_x_, kd_lin_x_, kp_ang_z_, ki_ang_z_, kd_ang_z_);
  }

  void setupCommunication() {
    // Wait for /clock if using sim time
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /clock to be active...");
        while (rclcpp::ok() && this->now().nanoseconds() == 0) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "/clock is active.");
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_in_, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        desired_ = *msg;
        last_cmd_time_ = this->now();
      }
    );

    fb_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      feedback_topic_, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        // If you want to use TwistStamped for feedback, you must construct it:
        feedback_.header = msg->header;
        feedback_.twist = msg->twist.twist;
        last_fb_time_ = this->now();
      }
    );

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_out_, 10);

    double period = 1.0 / control_frequency_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&TwistPIDController::controlLoop, this)
    );
  }

  void controlLoop() {
    auto now_time = this->now();
    double dt = std::max((now_time - last_fb_time_).seconds(), 1e-6);

    // this is optional but i'll leave it for now lol
    geometry_msgs::msg::Twist target, actual;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target = desired_.twist;
      actual = feedback_.twist;
    }

    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = this->now();
    output.header.frame_id = desired_.header.frame_id; // or set as needed

    {
      std::lock_guard<std::mutex> lock(mutex_);
      target = desired_.twist;
      actual = feedback_.twist;
    }

    output.twist.linear.x  = pid_lin_x_.compute(target.linear.x,  actual.linear.x,  dt);
    output.twist.linear.y  = pid_lin_y_.compute(target.linear.y,  actual.linear.y,  dt);
    output.twist.linear.z  = pid_lin_z_.compute(target.linear.z,  actual.linear.z,  dt);
    output.twist.angular.x = pid_ang_x_.compute(target.angular.x, actual.angular.x, dt);
    output.twist.angular.y = pid_ang_y_.compute(target.angular.y, actual.angular.y, dt);
    output.twist.angular.z = pid_ang_z_.compute(target.angular.z, actual.angular.z, dt);

    // could allow for indipendent feed-forward control, but whatever, if someone wants it they can add it
    if (feed_forward_) {
      output.twist.linear.x  += target.linear.x;
      output.twist.linear.y  += target.linear.y;
      output.twist.linear.z  += target.linear.z;
      output.twist.angular.x += target.angular.x;
      output.twist.angular.y += target.angular.y;
      output.twist.angular.z += target.angular.z;
    }

    cmd_pub_->publish(output);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPIDController>());
  rclcpp::shutdown();
  return 0;
}
