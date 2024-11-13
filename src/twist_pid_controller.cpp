#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "twist_pid_controller/msg/pid_debug.hpp"

using namespace std::chrono_literals;

class TwistPIDController : public rclcpp::Node
{
public:
  TwistPIDController()
  : Node("twist_pid_controller")
  {
    // Declare and get parameters
    this->declare_parameter<double>("kp_linear_x", 1.0);
    this->declare_parameter<double>("ki_linear_x", 0.0);
    this->declare_parameter<double>("kd_linear_x", 0.0);

    this->declare_parameter<double>("kp_linear_y", 1.0);
    this->declare_parameter<double>("ki_linear_y", 0.0);
    this->declare_parameter<double>("kd_linear_y", 0.0);

    this->declare_parameter<double>("kp_linear_z", 1.0);
    this->declare_parameter<double>("ki_linear_z", 0.0);
    this->declare_parameter<double>("kd_linear_z", 0.0);

    this->declare_parameter<double>("kp_angular_x", 1.0);
    this->declare_parameter<double>("ki_angular_x", 0.0);
    this->declare_parameter<double>("kd_angular_x", 0.0);

    this->declare_parameter<double>("kp_angular_y", 1.0);
    this->declare_parameter<double>("ki_angular_y", 0.0);
    this->declare_parameter<double>("kd_angular_y", 0.0);

    this->declare_parameter<double>("kp_angular_z", 1.0);
    this->declare_parameter<double>("ki_angular_z", 0.0);
    this->declare_parameter<double>("kd_angular_z", 0.0);

    this->declare_parameter<bool>("feed_forward", true);

    this->declare_parameter<double>("max_integral_linear", 1.0);
    this->declare_parameter<double>("max_integral_angular", 1.0);

    this->declare_parameter<double>("control_frequency", 50.0); // Control loop frequency in Hz
    this->declare_parameter<double>("timeout", 0.5); // Timeout in seconds

    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<std::string>("debug_topic", "pid_debug");
    this->declare_parameter<double>("k_increment", 0.1);

    this->declare_parameter<std::string>("cmd_vel_input_topic", "cmd_vel");
    this->declare_parameter<std::string>("feedback_vel_topic", "feedback_vel");
    this->declare_parameter<std::string>("cmd_vel_out_topic", "cmd_vel_out");
    this->declare_parameter<std::string>("joy_topic", "joy");

    this->get_parameter("kp_linear_x", kp_linear_x_);
    this->get_parameter("ki_linear_x", ki_linear_x_);
    this->get_parameter("kd_linear_x", kd_linear_x_);

    this->get_parameter("kp_linear_y", kp_linear_y_);
    this->get_parameter("ki_linear_y", ki_linear_y_);
    this->get_parameter("kd_linear_y", kd_linear_y_);

    this->get_parameter("kp_linear_z", kp_linear_z_);
    this->get_parameter("ki_linear_z", ki_linear_z_);
    this->get_parameter("kd_linear_z", kd_linear_z_);

    this->get_parameter("kp_angular_x", kp_angular_x_);
    this->get_parameter("ki_angular_x", ki_angular_x_);
    this->get_parameter("kd_angular_x", kd_angular_x_);

    this->get_parameter("kp_angular_y", kp_angular_y_);
    this->get_parameter("ki_angular_y", ki_angular_y_);
    this->get_parameter("kd_angular_y", kd_angular_y_);

    this->get_parameter("kp_angular_z", kp_angular_z_);
    this->get_parameter("ki_angular_z", ki_angular_z_);
    this->get_parameter("kd_angular_z", kd_angular_z_);

    this->get_parameter("feed_forward", feed_forward_);

    this->get_parameter("max_integral_linear", max_integral_linear_);
    this->get_parameter("max_integral_angular", max_integral_angular_);

    double control_frequency;
    this->get_parameter("control_frequency", control_frequency);
    this->get_parameter("timeout", timeout_);
    control_period_ = std::chrono::duration<double>(1.0 / control_frequency);

    this->get_parameter("debug", debug_);
    this->get_parameter("debug_topic", debug_topic_);
    this->get_parameter("k_increment", k_increment_);

    this->get_parameter("cmd_vel_input_topic", cmd_vel_in_);
    this->get_parameter("feedback_vel_topic", feedback_vel_);
    this->get_parameter("cmd_vel_out_topic", cmd_vel_out_);
    this->get_parameter("joy_topic", joy_topic_);

    // Initialize variables
    previous_time_ = this->now();

    // // Subscribers and Publishers
    cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_in_, 10,
      std::bind(&TwistPIDController::cmdCallback, this, std::placeholders::_1));

    feedback_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      feedback_vel_, 10,
      std::bind(&TwistPIDController::feedbackCallback, this, std::placeholders::_1));

    pid_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_, 10);

    if (debug_) {
      debug_publisher_ = this->create_publisher<twist_pid_controller::msg::PidDebug>(debug_topic_, 10);
      joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10,
        std::bind(&TwistPIDController::joyCallback, this, std::placeholders::_1));
    }

    // Timer for control loop
    control_timer_ = this->create_wall_timer(
      control_period_,
      std::bind(&TwistPIDController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Twist PID Controller node has been started");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    desired_twist_ = *msg;
    last_cmd_time_ = this->now();
  }

  void feedbackCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_twist_ = *msg;
    last_feedback_time_ = this->now();
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    joy_msg_ = *msg;
  }

  template <typename T> int getSgn(T val) {
      return (T(0) < val) - (val < T(0));
  }

  void controlLoop()
  {
    // Measure dt
    auto current_time = this->now();
    double dt = (current_time - previous_time_).seconds();
    previous_time_ = current_time;

    if (dt <= 0.0) {
      dt = 1e-6;  // Prevent division by zero or negative time
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Control loop period is 0 or negative, setting to 1e-6");
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
    if ((getSgn(desired.linear.x) != getSgn(previous_desired_linear_x_)) || (abs(desired.linear.x) < 0.05)) {
        integral_linear_x_ = 0.0;
    }
    if ((getSgn(desired.linear.y) != getSgn(previous_desired_linear_y_)) || (abs(desired.linear.y) < 0.05)) {
        integral_linear_y_ = 0.0;
    }
    if ((getSgn(desired.linear.z) != getSgn(previous_desired_linear_z_)) || (abs(desired.linear.z) < 0.05)) {
        integral_linear_z_ = 0.0;
    }
    if ((getSgn(desired.angular.x) != getSgn(previous_desired_angular_x_)) || (abs(desired.angular.x) < 0.05)) {
        integral_angular_x_ = 0.0;
    }
    if ((getSgn(desired.angular.y) != getSgn(previous_desired_angular_y_)) || (abs(desired.angular.y) < 0.05)) {
        integral_angular_y_ = 0.0;
    }
    if ((getSgn(desired.angular.z) != getSgn(previous_desired_angular_z_)) || (abs(desired.angular.z) < 0.05)) {
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
    control_output.linear.x = kp_linear_x_ * error_linear_x + ki_linear_x_ * integral_linear_x_ + kd_linear_x_ * derivative_linear_x;
    control_output.linear.y = kp_linear_y_ * error_linear_y + ki_linear_y_ * integral_linear_y_ + kd_linear_y_ * derivative_linear_y;
    control_output.linear.z = kp_linear_z_ * error_linear_z + ki_linear_z_ * integral_linear_z_ + kd_linear_z_ * derivative_linear_z;

    // Angular velocities
    control_output.angular.x = kp_angular_x_ * error_angular_x + ki_angular_x_ * integral_angular_x_ + kd_angular_x_ * derivative_angular_x;
    control_output.angular.y = kp_angular_y_ * error_angular_y + ki_angular_y_ * integral_angular_y_ + kd_angular_y_ * derivative_angular_y;
    control_output.angular.z = kp_angular_z_ * error_angular_z + ki_angular_z_ * integral_angular_z_ + kd_angular_z_ * derivative_angular_z;

    if (feed_forward_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Feed forward is enabled, make sure gains are not too high");
      control_output.linear.x += desired.linear.x;
      control_output.linear.y += desired.linear.y;
      control_output.linear.z += desired.linear.z;

      control_output.angular.x += desired.angular.x;
      control_output.angular.y += desired.angular.y;
      control_output.angular.z += desired.angular.z;
    }

    // Check if the command or feedback is too old
    if ((current_time - last_cmd_time_).seconds() > timeout_ || (current_time - last_feedback_time_).seconds() > timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Command or feedback is too old, sending zero commands");
      control_output.linear.x = 0.0;
      control_output.linear.y = 0.0;
      control_output.linear.z = 0.0;
      control_output.angular.x = 0.0;
      control_output.angular.y = 0.0;
      control_output.angular.z = 0.0;
    }

    // Publish the control output
    pid_publisher_->publish(control_output);

    // Publish debug information
    if (debug_) {

      RCLCPP_WARN_ONCE(this->get_logger(), "Debug mode is enabled");

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
      debug_msg.kp_linear.x = kp_linear_x_;
      debug_msg.kp_linear.y = kp_linear_y_;
      debug_msg.kp_linear.z = kp_linear_z_;

      debug_msg.ki_linear.x = ki_linear_x_;
      debug_msg.ki_linear.y = ki_linear_y_;
      debug_msg.ki_linear.z = ki_linear_z_;

      debug_msg.kd_linear.x = kd_linear_x_;
      debug_msg.kd_linear.y = kd_linear_y_;
      debug_msg.kd_linear.z = kd_linear_z_;

      debug_msg.kp_angular.x = kp_angular_x_;
      debug_msg.kp_angular.y = kp_angular_y_;
      debug_msg.kp_angular.z = kp_angular_z_;

      debug_msg.ki_angular.x = ki_angular_x_;
      debug_msg.ki_angular.y = ki_angular_y_;
      debug_msg.ki_angular.z = ki_angular_z_;

      debug_msg.kd_angular.x = kd_angular_x_;
      debug_msg.kd_angular.y = kd_angular_y_;
      debug_msg.kd_angular.z = kd_angular_z_;

      debug_publisher_->publish(debug_msg);

      if (!joy_msg_.buttons.empty() && (joy_msg_.buttons[KP_BUTTON_X] || joy_msg_.buttons[KP_BUTTON_Y] || joy_msg_.buttons[KP_BUTTON_ANG_Z]) && !button_pressed_) {
        if (joy_msg_.buttons[KP_BUTTON_X]) {
          if (joy_msg_.buttons[INCREASE]) {
            kp_linear_x_ += k_increment_;
            button_pressed_ = true;
          } else if (joy_msg_.buttons[DECREASE]) {
            kp_linear_x_ -= k_increment_;
            button_pressed_ = true;
          }
        } else if (joy_msg_.buttons[KP_BUTTON_Y]) {
          if (joy_msg_.buttons[INCREASE]) {
            kp_linear_y_ += k_increment_;
            button_pressed_ = true;
          } else if (joy_msg_.buttons[DECREASE]) {
            kp_linear_y_ -= k_increment_;
            button_pressed_ = true;
          }
        } else if (joy_msg_.buttons[KP_BUTTON_ANG_Z]) {
          if (joy_msg_.buttons[INCREASE]) {
            kp_angular_z_ += k_increment_;
            button_pressed_ = true;
          } else if (joy_msg_.buttons[DECREASE]) {
            kp_angular_z_ -= k_increment_;
            button_pressed_ = true;
          }
        }
      } else if (!joy_msg_.buttons.empty() && (joy_msg_.buttons[INCREASE] || joy_msg_.buttons[DECREASE])) {
        return;
      } else {
        button_pressed_ = false;
      }

      // Clamp the gains to positive values
      kp_linear_x_ = std::max(0.0, kp_linear_x_);
      ki_linear_x_ = std::max(0.0, ki_linear_x_);
      kd_linear_x_ = std::max(0.0, kd_linear_x_);

      kp_linear_y_ = std::max(0.0, kp_linear_y_);
      ki_linear_y_ = std::max(0.0, ki_linear_y_);
      kd_linear_y_ = std::max(0.0, kd_linear_y_);

      kp_linear_z_ = std::max(0.0, kp_linear_z_);
      ki_linear_z_ = std::max(0.0, ki_linear_z_);
      kd_linear_z_ = std::max(0.0, kd_linear_z_);

      kp_angular_x_ = std::max(0.0, kp_angular_x_);
      ki_angular_x_ = std::max(0.0, ki_angular_x_);
      kd_angular_x_ = std::max(0.0, kd_angular_x_);

      kp_angular_y_ = std::max(0.0, kp_angular_y_);
      ki_angular_y_ = std::max(0.0, ki_angular_y_);
      kd_angular_y_ = std::max(0.0, kd_angular_y_);

      kp_angular_z_ = std::max(0.0, kp_angular_z_);
      ki_angular_z_ = std::max(0.0, ki_angular_z_);
      kd_angular_z_ = std::max(0.0, kd_angular_z_);
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
  double kp_linear_x_{0.0};
  double ki_linear_x_{0.0};
  double kd_linear_x_{0.0};

  double kp_linear_y_{0.0};
  double ki_linear_y_{0.0};
  double kd_linear_y_{0.0};

  double kp_linear_z_{0.0};
  double ki_linear_z_{0.0};
  double kd_linear_z_{0.0};

  double kp_angular_x_{0.0};
  double ki_angular_x_{0.0};
  double kd_angular_x_{0.0};

  double kp_angular_y_{0.0}; 
  double ki_angular_y_{0.0}; 
  double kd_angular_y_{0.0};

  double kp_angular_z_{0.0};
  double ki_angular_z_{0.0};
  double kd_angular_z_{0.0}; 

  bool feed_forward_{false};

  double max_integral_linear_{0.0};
  double max_integral_angular_{0.0};

  std::string cmd_vel_in_;
  std::string feedback_vel_;
  std::string cmd_vel_out_;

  std::chrono::duration<double> control_period_;

  bool debug_{false};
  double k_increment_{0.0};
  std::string debug_topic_;
  std::string joy_topic_;

  // State variables
  rclcpp::Time previous_time_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_feedback_time_;
  double timeout_{0.0};

  double integral_linear_x_{0.0};
  double integral_linear_y_{0.0};
  double integral_linear_z_{0.0};

  double integral_angular_x_{0.0};
  double integral_angular_y_{0.0};
  double integral_angular_z_{0.0};

  double previous_error_linear_x_{0.0};
  double previous_error_linear_y_{0.0};
  double previous_error_linear_z_{0.0};

  double previous_error_angular_x_{0.0};
  double previous_error_angular_y_{0.0};
  double previous_error_angular_z_{0.0};

  double previous_desired_linear_x_{0.0};
  double previous_desired_linear_y_{0.0};
  double previous_desired_linear_z_{0.0};

  double previous_desired_angular_x_{0.0};
  double previous_desired_angular_y_{0.0};
  double previous_desired_angular_z_{0.0};

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr feedback_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pid_publisher_;

  // Debug publisher
  rclcpp::Publisher<twist_pid_controller::msg::PidDebug>::SharedPtr debug_publisher_;

  // Control loop timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Mutexes for thread safety
  std::mutex command_mutex_;
  std::mutex feedback_mutex_;
  std::mutex joy_mutex_;

  // Commanded and feedback twists
  geometry_msgs::msg::Twist desired_twist_;
  geometry_msgs::msg::Twist feedback_twist_;
  sensor_msgs::msg::Joy joy_msg_;

  // Debug message
  int INCREASE = 11;
  int DECREASE = 12;
  int KP_BUTTON_X = 2;
  int KP_BUTTON_Y = 3;
  int KP_BUTTON_ANG_Z = 1;

  bool button_pressed_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TwistPIDController>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
