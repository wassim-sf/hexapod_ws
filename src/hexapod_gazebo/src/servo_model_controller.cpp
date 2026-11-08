// The single command path: std_msgs/Float64MultiArray with 18 joint targets on /joint_commands.
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hexapod_gazebo/servo_model.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hexapod_gazebo
{
using Array = std_msgs::msg::Float64MultiArray;

class ServoModelController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<double>("kp", 12.0);
    auto_declare<double>("kd", 0.1);
    auto_declare<double>("stall_torque", 1.08);
    auto_declare<double>("no_load_speed", 7.48);
    auto_declare<double>("deadband", 0.0);
    auto_declare<double>("command_delay", 0.0);
    auto_declare<double>("response_tau", 0.04);
    auto_declare<std::string>("command_topic", "/joint_commands");
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, names("effort")};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> all;
    for (const auto & joint : joints_) {
      all.push_back(joint + "/position");
      all.push_back(joint + "/velocity");
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, all};
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    if (joints_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No joints configured");
      return CallbackReturn::ERROR;
    }
    params_.kp = get_node()->get_parameter("kp").as_double();
    params_.kd = get_node()->get_parameter("kd").as_double();
    params_.stall_torque = get_node()->get_parameter("stall_torque").as_double();
    params_.no_load_speed = get_node()->get_parameter("no_load_speed").as_double();
    params_.deadband = get_node()->get_parameter("deadband").as_double();
    delay_ = get_node()->get_parameter("command_delay").as_double();
    response_tau_ = get_node()->get_parameter("response_tau").as_double();
    subscription_ = get_node()->create_subscription<Array>(
      get_node()->get_parameter("command_topic").as_string(), rclcpp::SystemDefaultsQoS(),
      [this](const Array::SharedPtr msg) {
        if (msg->data.size() != joints_.size()) {
          RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "/joint_commands has %zu values, expected %zu. Ignored.", msg->data.size(), joints_.size());
          return;
        }
        for (const double value : msg->data) {
          if (!std::isfinite(value)) {
            RCLCPP_ERROR_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 2000, "Non-finite command ignored.");
            return;
          }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back({get_node()->now().seconds(), msg->data});
      });
    effort_publisher_ = get_node()->create_publisher<Array>("/servo_model/efforts", rclcpp::SystemDefaultsQoS());
    effort_rt_ = std::make_unique<realtime_tools::RealtimePublisher<Array>>(effort_publisher_);
    effort_rt_->msg_.data.assign(joints_.size(), 0.0);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Hold where the joints are until the first command arrives.
    targets_.resize(joints_.size());
    filtered_.resize(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      targets_[i] = state_interfaces_[2 * i].get_optional().value_or(0.0);
      filtered_[i] = targets_[i];
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    for (auto & command : command_interfaces_) {
      (void)command.set_value(0.0);
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    {
      // Apply the newest command that has waited out the transport delay.
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        const double now = time.seconds();
        while (!queue_.empty() && now - queue_.front().stamp >= delay_) {
          targets_ = queue_.front().values;
          queue_.pop_front();
        }
      }
    }
    if (effort_rt_) {
      publish_count_ = (publish_count_ + 1) % 10;
    }
    // First-order lag on the target: the servo's electronics and gear train do not respond instantly.
    const double dt = period.seconds();
    const double alpha = response_tau_ > 0.0 ? dt / (response_tau_ + dt) : 1.0;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      filtered_[i] += alpha * (targets_[i] - filtered_[i]);
      const double position = state_interfaces_[2 * i].get_optional().value_or(targets_[i]);
      const double velocity = state_interfaces_[2 * i + 1].get_optional().value_or(0.0);
      const double torque = servo_torque(params_, filtered_[i], position, velocity);
      (void)command_interfaces_[i].set_value(torque);
      if (effort_rt_) {
        effort_rt_->msg_.data[i] = torque;
      }
    }
    if (publish_count_ == 0 && effort_rt_ && effort_rt_->trylock()) {
      effort_rt_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  }

private:
  struct Timed
  {
    double stamp;
    std::vector<double> values;
  };

  std::vector<std::string> names(const std::string & interface) const
  {
    std::vector<std::string> out;
    for (const auto & joint : joints_) {
      out.push_back(joint + "/" + interface);
    }
    return out;
  }

  std::vector<std::string> joints_;
  std::vector<double> targets_;
  std::vector<double> filtered_;
  double response_tau_{0.04};
  ServoParams params_;
  double delay_{0.0};
  int publish_count_{0};
  std::mutex mutex_;
  std::deque<Timed> queue_;
  rclcpp::Subscription<Array>::SharedPtr subscription_;
  rclcpp::Publisher<Array>::SharedPtr effort_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Array>> effort_rt_;
};
}  // namespace hexapod_gazebo

PLUGINLIB_EXPORT_CLASS(hexapod_gazebo::ServoModelController, controller_interface::ControllerInterface)
