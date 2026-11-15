#ifndef BUMPERBOT_INTERFACE_HPP
#define BUMPERBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>

#include <vector>
#include <string>
#include <sstream>


namespace bumperbot_firmware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class BumperbotInterface : public hardware_interface::SystemInterface
    {
    public:
        BumperbotInterface();
        virtual ~BumperbotInterface();

        CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        bool parseTelemetry(const std::string & line);

        LibSerial::SerialPort arduino_;
        std::string port_;
        double cpr_{1320.0};
        double rad_per_count_{0.0};
        int right_idx_{-1};
        int left_idx_{-1};

        std::vector<double> velocity_commands_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;
    };
} // namespace bumperbot_firmware

#endif // BUMPERBOT_FIRMWARE_HPP