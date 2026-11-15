#include "bumperbot_firmware/bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace bumperbot_firmware
{
static rclcpp::Logger logger() { return rclcpp::get_logger("BumperbotInterface"); }

BumperbotInterface::BumperbotInterface() {}

BumperbotInterface::~BumperbotInterface()
{
  if (arduino_.IsOpen()) {
    try { arduino_.Close(); }
    catch (...) { RCLCPP_FATAL_STREAM(logger(), "Error closing port " << port_); }
  }
}

CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  try {
    port_ = info_.hardware_parameters.at("port");
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(logger(), "No serial port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  auto it = info_.hardware_parameters.find("counts_per_rev");
  if (it != info_.hardware_parameters.end()) {
    try { cpr_ = std::stod(it->second); }
    catch (...) {
      RCLCPP_FATAL(logger(), "Invalid counts_per_rev");
      return CallbackReturn::FAILURE;
    }
  }
  rad_per_count_ = 2.0 * M_PI / cpr_;

  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(logger(), "Expected exactly 2 joints");
    return CallbackReturn::FAILURE;
  }
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (info_.joints[i].name == "wheel_right_joint") right_idx_ = static_cast<int>(i);
    if (info_.joints[i].name == "wheel_left_joint") left_idx_ = static_cast<int>(i);
  }
  if (right_idx_ < 0 || left_idx_ < 0) {
    RCLCPP_FATAL(logger(), "Joints must be named wheel_right_joint and wheel_left_joint");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.assign(2, 0.0);
  position_states_.assign(2, 0.0);
  velocity_states_.assign(2, 0.0);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger(), "Starting robot hardware on %s ...", port_.c_str());
  velocity_commands_.assign(2, 0.0);
  position_states_.assign(2, 0.0);
  velocity_states_.assign(2, 0.0);

  try {
    if (arduino_.IsOpen()) { arduino_.Close(); }
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  } catch (...) {
    RCLCPP_FATAL_STREAM(logger(), "Could not open port " << port_);
    return CallbackReturn::FAILURE;
  }

  // Opening the port resets the Nano: wait for the reboot, drop boot garbage
  std::this_thread::sleep_for(std::chrono::seconds(2));
  arduino_.FlushIOBuffers();

  RCLCPP_INFO(logger(), "Hardware started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger(), "Stopping robot hardware ...");
  velocity_commands_.assign(2, 0.0);
  if (arduino_.IsOpen()) {
    try {
      arduino_.Write("V 0 0\n");
      arduino_.DrainWriteBuffer();
      arduino_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(logger(), "Error while closing port " << port_);
      return CallbackReturn::FAILURE;
    }
  }
  return CallbackReturn::SUCCESS;
}

// Line example: R_vel:10.02,L_vel:9.98,R_cnt:12345,L_cnt:12340
bool BumperbotInterface::parseTelemetry(const std::string & line)
{
  double r_vel = 0, l_vel = 0, r_cnt = 0, l_cnt = 0;
  unsigned got = 0;
  std::stringstream ss(line);
  std::string token;
  try {
    while (std::getline(ss, token, ',')) {
      auto pos = token.find(':');
      if (pos == std::string::npos) continue;
      const std::string key = token.substr(0, pos);
      const double val = std::stod(token.substr(pos + 1));
      if (key == "R_vel") { r_vel = val; got |= 1; }
      else if (key == "L_vel") { l_vel = val; got |= 2; }
      else if (key == "R_cnt") { r_cnt = val; got |= 4; }
      else if (key == "L_cnt") { l_cnt = val; got |= 8; }
    }
  } catch (const std::exception &) {
    return false;  // garbled line (boot message, torn read)
  }
  if (got != 0xF) return false;

  velocity_states_[right_idx_] = r_vel;
  velocity_states_[left_idx_] = l_vel;
  position_states_[right_idx_] = r_cnt * rad_per_count_;
  position_states_[left_idx_] = l_cnt * rad_per_count_;
  return true;
}

hardware_interface::return_type BumperbotInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!arduino_.IsOpen()) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    // Drain the buffer and keep only the newest complete line
    std::string line, latest;
    while (arduino_.IsDataAvailable()) {
      arduino_.ReadLine(line, '\n', 5);  // 5 ms timeout
      latest = line;
    }
    if (!latest.empty()) {
      parseTelemetry(latest);
    }
  } catch (const LibSerial::ReadTimeout &) {
    // partial line, the next cycle will catch up
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger(), "Error reading from " << port_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BumperbotInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!arduino_.IsOpen()) {
    return hardware_interface::return_type::ERROR;
  }
  char msg[48];
  std::snprintf(msg, sizeof(msg), "V %.2f %.2f\n",
    velocity_commands_[right_idx_], velocity_commands_[left_idx_]);
  try {
    arduino_.Write(msg);
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger(), "Error sending " << msg << " on " << port_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}
}  // namespace bumperbot_firmware

PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::BumperbotInterface, hardware_interface::SystemInterface)