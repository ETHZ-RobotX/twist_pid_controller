#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "twist_pid_controller/msg/pid_debug.hpp"

using namespace std::chrono_literals;

class TwistPIDController : public rclcpp::Node
{
public:
  TwistPIDController(const rclcpp::NodeOptions & options)
  : Node("twist_pid_controller", options)
  {
    // Declare and get parameters
    this->declare_parameter<double>("kp_linear", 1.0);
    this->declare_parameter<double>("ki_linear", 0.0);
    this->declare_parameter<double>("kd_linear", 0.0);

    this->declare_parameter<double>("kp_angular", 1.0);
    this->declare_parameter<double>("ki_angular", 0.0);
    this->declare_parameter<double>("kd_angular", 0.0);

    this->declare_parameter<double>("max_integral_linear", 1.0);
    this->declare_parameter<double>("max_integral_angular", 1.0);

    this->declare_parameter<double>("control_frequency", 50.0); // Control loop frequency in Hz

    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<std::string>("debug_topic", "pid_debug");

    this->get_parameter("kp_linear", kp_linear_);
    this->get_parameter("ki_linear", ki_linear_);
    this->get_parameter("kd_linear", kd_linear_);

    this->get_parameter("kp_angular", kp_angular_);
    this->get_parameter("ki_angular", ki_angular_);
    this->get_parameter("kd_angular", kd_angular_);

    this->get_parameter("max_integral_linear", max_integral_linear_);
    this->get_parameter("max_integral_angular", max_integral_angular_);

    double control_frequency;
    this->get_parameter("control_frequency", control_frequency);
    control_period_ = rclcpp::Duration::from_seconds(1.0 / control_frequency);

    this->get_parameter("debug", debug_);
    this->get_parameter("debug_topic", debug_topic_);

    // Initialize variables
    previous_time_ = this->now();
    integral_linear_x_ = 0.0;
    integral_linear_y_ = 0.0;
    integral_linear_z_ = 0.0;
    integral_angular_x_ = 0.0;
    integral_angular_y_ = 0.0;
    integral_angular_z_ = 0.0;

    previous_error_linear_x_ = 0.0;
    previous_error_linear_y_ = 0.0;
    previous_error_linear_z_ = 0.0;
    previous_error_angular_x_ = 0.0;
    previous_error_angular_y_ = 0.0;
    previous_error_angular_z_ = 0.0;

    previous_desired_linear_x_ = 0.0;
    previous_desired_linear_y_ = 0.0;
    previous_desired_linear_z_ = 0.0;
    previous_desired_angular_x_ = 0.0;
    previous_desired_angular_y_ = 0.0;
    previous_desired_angular_z_ = 0.0;

    // Subscribers and Publishers
    cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", 10,
      std::bind(&TwistPIDController::cmdCallback, this, std::placeholders::_1));

    feedback_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "feedback_vel", 10,
      std::bind(&TwistPIDController::feedbackCallback, this, std::placeholders::_1));

    pid_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);

    if (debug_) {
      debug_publisher_ = this->create_publisher<twist_pid_controller::msg::PidDebug>(debug_topic_, 10);
    }

    // Timer for control loop
    control_timer_ = this->create_wall_timer(
      control_period_,
      std::bind(&TwistPIDController::controlLoop, this));
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    desired_twist_ = *msg;
  }

  void feedbackCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_twist_ = *msg;
  }

  void controlLoop()
  {
    // Measure dt
    auto current_time = this->now();
    double dt = (current_time - previous_time_).seconds();
    previous_time_ = current_time;

    if (dt <= 0.0) {
      dt = 1e-6;  // Prevent division by zero or negative time
    }

    // Get desired and feedback velocities
    geometry_msgs::msg::Twist desired;
    geometry_msgs::msg::Twist feedback;

    {
      std::lock_guard<std::mutex> lock1(command_mutex_);
      desired = desired_twist_;
    }
    {
      std::lock_guard<std::mutex> lock2(feedback_mutex_);
      feedback = feedback_twist_;
    }

    // Compute errors
    double error_linear_x = desired.linear.x - feedback.linear.x;
    double error_linear_y = desired.linear.y - feedback.linear.y;
    double error_linear_z = desired.linear.z - feedback.linear.z;

    double error_angular_x = desired.angular.x - feedback.angular.x;
    double error_angular_y = desired.angular.y - feedback.angular.y;
    double error_angular_z = desired.angular.z - feedback.angular.z;

    // Reset integrator if command changes direction or braking is desired
    if ((desired.linear.x * previous_desired_linear_x_ < 0.0) || (desired.linear.x == 0.0)) {
      integral_linear_x_ = 0.0;
    }
    if ((desired.linear.y * previous_desired_linear_y_ < 0.0) || (desired.linear.y == 0.0)) {
      integral_linear_y_ = 0.0;
    }
    if ((desired.linear.z * previous_desired_linear_z_ < 0.0) || (desired.linear.z == 0.0)) {
      integral_linear_z_ = 0.0;
    }
    if ((desired.angular.x * previous_desired_angular_x_ < 0.0) || (desired.angular.x == 0.0)) {
      integral_angular_x_ = 0.0;
    }
    if ((desired.angular.y * previous_desired_angular_y_ < 0.0) || (desired.angular.y == 0.0)) {
      integral_angular_y_ = 0.0;
    }
    if ((desired.angular.z * previous_desired_angular_z_ < 0.0) || (desired.angular.z == 0.0)) {
      integral_angular_z_ = 0.0;
    }

    // Update previous desired velocities
    previous_desired_linear_x_ = desired.linear.x;
    previous_desired_linear_y_ = desired.linear.y;
    previous_desired_linear_z_ = desired.linear.z;

    previous_desired_angular_x_ = desired.angular.x;
    previous_desired_angular_y_ = desired.angular.y;
    previous_desired_angular_z_ = desired.angular.z;

    // Integrate errors
    integral_linear_x_ += error_linear_x * dt;
    integral_linear_y_ += error_linear_y * dt;
    integral_linear_z_ += error_linear_z * dt;

    integral_angular_x_ += error_angular_x * dt;
    integral_angular_y_ += error_angular_y * dt;
    integral_angular_z_ += error_angular_z * dt;

    // Clamp integrators
    integral_linear_x_ = clamp(integral_linear_x_, -max_integral_linear_, max_integral_linear_);
    integral_linear_y_ = clamp(integral_linear_y_, -max_integral_linear_, max_integral_linear_);
    integral_linear_z_ = clamp(integral_linear_z_, -max_integral_linear_, max_integral_linear_);

    integral_angular_x_ = clamp(integral_angular_x_, -max_integral_angular_, max_integral_angular_);
    integral_angular_y_ = clamp(integral_angular_y_, -max_integral_angular_, max_integral_angular_);
    integral_angular_z_ = clamp(integral_angular_z_, -max_integral_angular_, max_integral_angular_);

    // Derivative terms
    double derivative_linear_x = (error_linear_x - previous_error_linear_x_) / dt;
    double derivative_linear_y = (error_linear_y - previous_error_linear_y_) / dt;
    double derivative_linear_z = (error_linear_z - previous_error_linear_z_) / dt;

    double derivative_angular_x = (error_angular_x - previous_error_angular_x_) / dt;
    double derivative_angular_y = (error_angular_y - previous_error_angular_y_) / dt;
    double derivative_angular_z = (error_angular_z - previous_error_angular_z_) / dt;

    // Update previous errors
    previous_error_linear_x_ = error_linear_x;
    previous_error_linear_y_ = error_linear_y;
    previous_error_linear_z_ = error_linear_z;

    previous_error_angular_x_ = error_angular_x;
    previous_error_angular_y_ = error_angular_y;
    previous_error_angular_z_ = error_angular_z;

    // Compute PID outputs
    geometry_msgs::msg::Twist control_output;

    // Linear velocities
    control_output.linear.x = kp_linear_ * error_linear_x + ki_linear_ * integral_linear_x_ + kd_linear_ * derivative_linear_x;
    control_output.linear.y = kp_linear_ * error_linear_y + ki_linear_ * integral_linear_y_ + kd_linear_ * derivative_linear_y;
    control_output.linear.z = kp_linear_ * error_linear_z + ki_linear_ * integral_linear_z_ + kd_linear_ * derivative_linear_z;

    // Angular velocities
    control_output.angular.x = kp_angular_ * error_angular_x + ki_angular_ * integral_angular_x_ + kd_angular_ * derivative_angular_x;
    control_output.angular.y = kp_angular_ * error_angular_y + ki_angular_ * integral_angular_y_ + kd_angular_ * derivative_angular_y;
    control_output.angular.z = kp_angular_ * error_angular_z + ki_angular_ * integral_angular_z_ + kd_angular_ * derivative_angular_z;

    // Publish the control output
    pid_publisher_->publish(control_output);

    // Publish debug information
    if (debug_) {
      auto debug_msg = twist_pid_controller::msg::PidDebug();

      // Errors
      debug_msg.error_linear.x = error_linear_x;
      debug_msg.error_linear.y = error_linear_y;
      debug_msg.error_linear.z = error_linear_z;

      debug_msg.error_angular.x = error_angular_x;
      debug_msg.error_angular.y = error_angular_y;
      debug_msg.error_angular.z = error_angular_z;

      // Integrals
      debug_msg.integral_linear.x = integral_linear_x_;
      debug_msg.integral_linear.y = integral_linear_y_;
      debug_msg.integral_linear.z = integral_linear_z_;

      debug_msg.integral_angular.x = integral_angular_x_;
      debug_msg.integral_angular.y = integral_angular_y_;
      debug_msg.integral_angular.z = integral_angular_z_;

      // Derivatives
      debug_msg.derivative_linear.x = derivative_linear_x;
      debug_msg.derivative_linear.y = derivative_linear_y;
      debug_msg.derivative_linear.z = derivative_linear_z;

      debug_msg.derivative_angular.x = derivative_angular_x;
      debug_msg.derivative_angular.y = derivative_angular_y;
      debug_msg.derivative_angular.z = derivative_angular_z;

      // PID Gains
      debug_msg.kp.x = kp_linear_;
      debug_msg.kp.y = kp_linear_;
      debug_msg.kp.z = kp_linear_;

      debug_msg.ki.x = ki_linear_;
      debug_msg.ki.y = ki_linear_;
      debug_msg.ki.z = ki_linear_;

      debug_msg.kd.x = kd_linear_;
      debug_msg.kd.y = kd_linear_;
      debug_msg.kd.z = kd_linear_;

      debug_publisher_->publish(debug_msg);
    }
  }

  // Helper function to clamp values
  double clamp(double value, double min_value, double max_value)
  {
    if (value > max_value) {
      return max_value;
    } else if (value < min_value) {
      return min_value;
    } else {
      return value;
    }
  }

  // Parameters
  double kp_linear_;
  double ki_linear_;
  double kd_linear_;

  double kp_angular_;
  double ki_angular_;
  double kd_angular_;

  double max_integral_linear_;
  double max_integral_angular_;

  rclcpp::Duration control_period_;

  bool debug_;
  std::string debug_topic_;

  // State variables
  rclcpp::Time previous_time_;

  double integral_linear_x_;
  double integral_linear_y_;
  double integral_linear_z_;

  double integral_angular_x_;
  double integral_angular_y_;
  double integral_angular_z_;

  double previous_error_linear_x_;
  double previous_error_linear_y_;
  double previous_error_linear_z_;

  double previous_error_angular_x_;
  double previous_error_angular_y_;
  double previous_error_angular_z_;

  double previous_desired_linear_x_;
  double previous_desired_linear_y_;
  double previous_desired_linear_z_;

  double previous_desired_angular_x_;
  double previous_desired_angular_y_;
  double previous_desired_angular_z_;

  // Subscribers and Publishers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr feedback_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pid_publisher_;

  // Debug publisher
  rclcpp::Publisher<twist_pid_controller::msg::PidDebug>::SharedPtr debug_publisher_;

  // Control loop timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Mutexes for thread safety
  std::mutex command_mutex_;
  std::mutex feedback_mutex_;

  // Commanded and feedback twists
  geometry_msgs::msg::Twist desired_twist_;
  geometry_msgs::msg::Twist feedback_twist_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a node with options to allow parameter overrides
  rclcpp::NodeOptions options;
  auto node = std::make_shared<TwistPIDController>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
