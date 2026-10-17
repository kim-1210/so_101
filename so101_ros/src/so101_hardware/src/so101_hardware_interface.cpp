#include "so101_hardware/so101_hardware_interface.hpp"

#include <array>
#include <exception>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "so101_hardware/packet.hpp"
#include "so101_hardware/serial_utils.hpp"

namespace
{

constexpr const char * kLoggerName = "So101HardwareInterface";

std::string get_parameter_or(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    const std::string & default_value)
{
    const auto iter = info.hardware_parameters.find(name);
    if (iter == info.hardware_parameters.end()) {
        return default_value;
    }

    return iter->second;
}

bool check_response_checksum_array(const std::array<uint8_t, so101_hardware::READ_FEEDBACK_SIZE + 6> & response)
{
    uint16_t sum = 0;

    for (std::size_t index = 2; index < response.size() - 1; ++index) {
        sum += response[index];
    }

    return static_cast<uint8_t>(~sum) == response.back();
}

}  // namespace

namespace so101_hardware
{

// ros2_control에서 전달한 하드웨어 정보와 파라미터를 읽고 내부 상태 배열을 초기화한다.
hardware_interface::CallbackReturn So101HardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints.size() != kJointCount) {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "SO101 needs exactly %zu joints, but got %zu",
            kJointCount,
            info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    arm_type_ = get_parameter_or(info_, "arm_type", "follower");
    port_name_ = get_parameter_or(info_, "port", DEFAULT_PORT);
    config_path_ = get_parameter_or(info_, "config_path", "");

    try {
        speed_ = static_cast<uint16_t>(std::stoi(get_parameter_or(info_, "speed", "300")));
        acc_ = static_cast<uint8_t>(std::stoi(get_parameter_or(info_, "acc", "20")));
        torque_limit_ = static_cast<uint16_t>(std::stoi(get_parameter_or(info_, "torque_limit", "300")));
        if (config_path_.empty()) {
            config_path_ = default_config_path_for_arm(arm_type_);
        }
        configs_ = load_motor_config(config_path_);
    } catch (const std::exception & error) {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "Failed to load SO101 hardware parameters: %s",
            error.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (arm_type_ == "leader") {
        motor_ids_ = {1, 2, 3, 4, 5, 6};
    } else if (arm_type_ == "follower") {
        motor_ids_ = {11, 12, 13, 14, 15, 16};
    } else {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "arm_type must be leader or follower, got: %s",
            arm_type_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.fill(0.0);
    hw_velocities_.fill(0.0);
    hw_efforts_.fill(0.0);
    hw_commands_.fill(0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// controller가 읽을 수 있는 관절 상태(position, velocity, effort)를 등록한다.
std::vector<hardware_interface::StateInterface>
So101HardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(kJointCount * 3);

    for (std::size_t index = 0; index < kJointCount; ++index) {
        const auto & joint_name = info_.joints[index].name;

        state_interfaces.emplace_back(
            joint_name,
            hardware_interface::HW_IF_POSITION,
            &hw_positions_[index]);
        state_interfaces.emplace_back(
            joint_name,
            hardware_interface::HW_IF_VELOCITY,
            &hw_velocities_[index]);
        state_interfaces.emplace_back(
            joint_name,
            hardware_interface::HW_IF_EFFORT,
            &hw_efforts_[index]);
    }

    return state_interfaces;
}

// controller가 쓸 수 있는 관절 명령(position)을 등록한다.
std::vector<hardware_interface::CommandInterface>
So101HardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(kJointCount);

    for (std::size_t index = 0; index < kJointCount; ++index) {
        command_interfaces.emplace_back(
            info_.joints[index].name,
            hardware_interface::HW_IF_POSITION,
            &hw_commands_[index]);
    }

    return command_interfaces;
}

// 하드웨어 사용 준비 단계에서 시리얼 포트를 연다.
hardware_interface::CallbackReturn So101HardwareInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    try {
        if (!serial_.is_open()) {
            configure_serial(serial_, port_name_, DEFAULT_BAUDRATE);
        }
    } catch (const std::exception & error) {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "Failed to open serial port %s: %s",
            port_name_.c_str(),
            error.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// 제어 시작 단계에서 실제 현재 위치를 읽고, 그 위치를 목표 위치로 맞춘 뒤 모터 토크를 켠다.
hardware_interface::CallbackReturn So101HardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    for (std::size_t index = 0; index < kJointCount; ++index) {
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;

        if (!read_motor_feedback(
                static_cast<uint8_t>(motor_ids_[index]),
                position,
                velocity,
                effort)) {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLoggerName),
                "Failed to read current position from motor ID %d during activation",
                motor_ids_[index]);
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_positions_[index] = position;
        hw_velocities_[index] = velocity;
        hw_efforts_[index] = effort;
        hw_commands_[index] = position;
    }

    try {
        for (std::size_t index = 0; index < kJointCount; ++index) {
            const int motor_id = motor_ids_[index];
            const auto id = static_cast<uint8_t>(motor_id);
            const int config_id = config_id_for_motor(arm_type_, motor_id);
            const auto & config = configs_.at(config_id);
            const uint16_t raw_goal = calibrated_rad_to_tick(hw_commands_[index], config);

            write_packet(serial_, make_torque_enable_packet(id, true));
            write_packet(serial_, make_torque_limit_packet(id, torque_limit_));
            write_packet(serial_, make_goal_packet(id, raw_goal, speed_, acc_));
        }
    } catch (const std::exception & error) {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "Failed to activate SO101 hardware: %s",
            error.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// 제어 종료 단계에서 모터 토크를 끄고 시리얼 포트를 닫는다.
hardware_interface::CallbackReturn So101HardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    try {
        for (const int motor_id : motor_ids_) {
            write_packet(serial_, make_torque_enable_packet(static_cast<uint8_t>(motor_id), false));
        }
    } catch (const std::exception & error) {
        RCLCPP_WARN(
            rclcpp::get_logger(kLoggerName),
            "Failed to deactivate all SO101 motors cleanly: %s",
            error.what());
    }

    if (serial_.is_open()) {
        serial_.close();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// 모터 하나의 현재 위치, 속도, 부하를 시리얼 통신으로 읽는다.
bool So101HardwareInterface::read_motor_feedback(
    uint8_t motor_id,
    double & position_rad,
    double & velocity_raw,
    double & effort_raw)
{
    write_packet(serial_, make_read_feedback_packet(motor_id), 0);

    std::array<uint8_t, READ_FEEDBACK_SIZE + 6> response{};
    boost::system::error_code error;
    const auto bytes_read = boost::asio::read(
        serial_,
        boost::asio::buffer(response),
        boost::asio::transfer_exactly(response.size()),
        error);

    if (error || bytes_read != response.size()) {
        RCLCPP_WARN(
            rclcpp::get_logger(kLoggerName),
            "ID %u: no response",
            motor_id);
        return false;
    }

    if (response[0] != 0xFF ||
        response[1] != 0xFF ||
        response[2] != motor_id ||
        response[4] != 0 ||
        !check_response_checksum_array(response)) {
        RCLCPP_WARN(
            rclcpp::get_logger(kLoggerName),
            "ID %u: invalid response",
            motor_id);
        return false;
    }

    const int raw_position = response[5] | (response[6] << 8);
    const uint16_t speed_raw = response[7] | (response[8] << 8);
    const uint16_t load_raw = response[9] | (response[10] << 8);

    const int config_id = config_id_for_motor(arm_type_, motor_id);
    const auto & config = configs_.at(config_id);

    position_rad = calibrated_tick_to_rad(raw_position, config);
    velocity_raw = static_cast<double>(decode_sign_magnitude_15(speed_raw));
    effort_raw = static_cast<double>(decode_sign_magnitude_15(load_raw));

    return true;
}

// 모든 모터 상태를 읽어서 ros2_control의 상태 배열에 저장한다.
hardware_interface::return_type So101HardwareInterface::read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    for (std::size_t index = 0; index < kJointCount; ++index) {
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;

        if (read_motor_feedback(
                static_cast<uint8_t>(motor_ids_[index]),
                position,
                velocity,
                effort)) {
            hw_positions_[index] = position;
            hw_velocities_[index] = velocity;
            hw_efforts_[index] = effort;
        }
    }

    return hardware_interface::return_type::OK;
}

// ros2_control이 준 목표 위치를 모터 tick 값으로 바꿔 모든 모터에 보낸다.
hardware_interface::return_type So101HardwareInterface::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    try {
        for (std::size_t index = 0; index < kJointCount; ++index) {
            const int motor_id = motor_ids_[index];
            const int config_id = config_id_for_motor(arm_type_, motor_id);
            const auto & config = configs_.at(config_id);

            // The Feetech motor already applies Homing_Offset internally.
            // Use LeRobot's calibrated range midpoint as ROS joint zero.
            const uint16_t raw_goal = calibrated_rad_to_tick(hw_commands_[index], config);

            const auto id = static_cast<uint8_t>(motor_id);
            write_packet(serial_, make_torque_enable_packet(id, true));
            write_packet(serial_, make_torque_limit_packet(id, torque_limit_));
            write_packet(serial_, make_goal_packet(id, raw_goal, speed_, acc_));
        }
    } catch (const std::exception & error) {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "Failed to write SO101 commands: %s",
            error.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace so101_hardware

PLUGINLIB_EXPORT_CLASS(
    so101_hardware::So101HardwareInterface,
    hardware_interface::SystemInterface)
