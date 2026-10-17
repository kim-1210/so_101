#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "so101_hardware/common.hpp"
#include "so101_hardware/packet.hpp"
#include "so101_hardware/serial_utils.hpp"

using namespace std::chrono_literals;

namespace so101_hardware
{

class So101HardwareNode : public rclcpp::Node
{
public:
    So101HardwareNode()
    : Node("so101_hardware_node"),
      serial_(io_)
    {
        arm_type_ = declare_parameter<std::string>("arm_type", "follower");
        port_name_ = declare_parameter<std::string>("port", DEFAULT_PORT);
        config_path_ = declare_parameter<std::string>("config_path", "");
        rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
        speed_ = static_cast<uint16_t>(declare_parameter<int>("speed", 300));
        acc_ = static_cast<uint8_t>(declare_parameter<int>("acc", 20));
        torque_limit_ = static_cast<uint16_t>(declare_parameter<int>("torque_limit", 300));
        enable_torque_ = declare_parameter<bool>("enable_torque", true);

        motor_ids_ = motor_ids_for_arm(arm_type_);
        if (config_path_.empty()) {
            config_path_ = default_config_path_for_arm(arm_type_);
        }
        configs_ = load_motor_config(config_path_);
        joint_names_ = make_joint_names();

        try {
            configure_serial(serial_, port_name_, DEFAULT_BAUDRATE);

            if (!enable_torque_) {
                disable_torque();
            } else if (arm_type_ == "follower") {
                hold_current_position();
            }
        } catch (const std::exception &error) {
            RCLCPP_FATAL(
                get_logger(),
                "Failed to start %s hardware on port %s: %s",
                arm_type_.c_str(),
                port_name_.c_str(),
                error.what());
            throw;
        }

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        if (enable_torque_) {
            command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
                "joint_commands",
                1,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    handle_joint_command(msg);
                });
        }

        auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        read_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            [this]() {
                publish_joint_states();
            });

        RCLCPP_INFO(
            get_logger(),
            "SO101 hardware node started. arm_type=%s port=%s enable_torque=%s",
            arm_type_.c_str(),
            port_name_.c_str(),
            enable_torque_ ? "true" : "false");
    }

private:
    std::vector<std::string> make_joint_names() const
    {
        return {
            "Rotation",
            "Pitch",
            "Elbow",
            "Wrist_Pitch",
            "Wrist_Roll",
            "Jaw"
        };
    }

    bool read_motor_feedback(
        int motor_id,
        double &position_rad,
        double &velocity_raw,
        double &effort_raw)
    {
        uint8_t id = static_cast<uint8_t>(motor_id);

        write_packet(serial_, make_read_feedback_packet(id), 0);

        std::vector<uint8_t> response;
        if (!read_bytes(serial_, response, READ_FEEDBACK_SIZE + 6)) {
            RCLCPP_WARN(get_logger(), "ID %d: no response", motor_id);
            return false;
        }

        if (response[0] != 0xFF || response[1] != 0xFF || response[2] != id ||
            response[4] != 0 || !check_response_checksum(response)) {
            RCLCPP_WARN(get_logger(), "ID %d: invalid response", motor_id);
            return false;
        }

        int raw_position = response[5] | (response[6] << 8);
        uint16_t speed_raw = response[7] | (response[8] << 8);
        uint16_t load_raw = response[9] | (response[10] << 8);

        int config_id = config_id_for_motor(arm_type_, motor_id);
        const auto &config = configs_.at(config_id);

        position_rad = calibrated_tick_to_rad(raw_position, config);
        velocity_raw = static_cast<double>(decode_sign_magnitude_15(speed_raw));
        effort_raw = static_cast<double>(decode_sign_magnitude_15(load_raw));

        return true;
    }

    void hold_current_position()
    {
        for (size_t index = 0; index < motor_ids_.size(); ++index) {
            int motor_id = motor_ids_[index];
            uint8_t id = static_cast<uint8_t>(motor_id);

            double position = 0.0;
            double velocity = 0.0;
            double effort = 0.0;

            if (!read_motor_feedback(motor_id, position, velocity, effort)) {
                RCLCPP_WARN(
                    get_logger(),
                    "ID %d: failed to hold current position on startup",
                    motor_id);
                continue;
            }

            int config_id = config_id_for_motor(arm_type_, motor_id);
            const auto &config = configs_.at(config_id);
            uint16_t raw_goal = calibrated_rad_to_tick(position, config);

            // write_packet(serial_, make_torque_enable_packet(id, true));
            // write_packet(serial_, make_torque_limit_packet(id, torque_limit_));
            write_packet(serial_, make_goal_packet(id, raw_goal, speed_, acc_) , 2);
        }

        RCLCPP_INFO(get_logger(), "Follower torque enabled at current position");
    }

    void disable_torque()
    {
        for (const int motor_id : motor_ids_) {
            uint8_t id = static_cast<uint8_t>(motor_id);
            write_packet(serial_, make_torque_enable_packet(id, false));
        }

        RCLCPP_INFO(get_logger(), "%s torque disabled", arm_type_.c_str());
    }

    void publish_joint_states()
    {
        sensor_msgs::msg::JointState msg;
        msg.name = joint_names_;
        msg.position.resize(motor_ids_.size());
        msg.velocity.resize(motor_ids_.size());
        msg.effort.resize(motor_ids_.size());

        for (size_t index = 0; index < motor_ids_.size(); ++index) {
            double position = 0.0;
            double velocity = 0.0;
            double effort = 0.0;

            if (!read_motor_feedback(motor_ids_[index], position, velocity, effort)) {
                // position 벡터는 resize 직후 0으로 초기화된다. 읽기에 실패한
                // 프레임을 그대로 publish하면 해당 관절이 순간적으로 0 rad로
                // 이동한 것처럼 robot_state_publisher가 잘못된 TF를 만든다.
                // 완전한 6관절 묶음을 읽은 프레임만 publish한다.
                RCLCPP_WARN(
                    get_logger(),
                    "Skipping incomplete JointState frame after motor ID %d read failure",
                    motor_ids_[index]);
                return;
            }

            msg.position[index] = position;
            msg.velocity[index] = velocity;
            msg.effort[index] = effort;
        }

        // 여섯 모터를 모두 읽은 뒤의 시각을 사용한다.
        msg.header.stamp = now();
        joint_state_pub_->publish(msg);
    }

    void handle_joint_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!enable_torque_) {
            RCLCPP_WARN(get_logger(), "Ignoring joint_commands because enable_torque is false");
            return;
        }

        if (msg->data.size() != motor_ids_.size()) {
            RCLCPP_WARN(
                get_logger(),
                "joint_commands needs %zu values, got %zu",
                motor_ids_.size(),
                msg->data.size());
            return;
        }

        for (size_t index = 0; index < motor_ids_.size(); ++index) {
            int motor_id = motor_ids_[index];
            uint8_t id = static_cast<uint8_t>(motor_id);

            int config_id = config_id_for_motor(arm_type_, motor_id);
            const auto &config = configs_.at(config_id);

            uint16_t raw_goal = calibrated_rad_to_tick(msg->data[index], config);

            // write_packet(serial_, make_torque_enable_packet(id, true));
            // write_packet(serial_, make_torque_limit_packet(id, torque_limit_));
            write_packet(serial_, make_goal_packet(id, raw_goal, speed_, acc_), 2);
        }
    }

    std::string arm_type_;
    std::string port_name_;
    std::string config_path_;
    double rate_hz_;
    uint16_t speed_;
    uint8_t acc_;
    uint16_t torque_limit_;
    bool enable_torque_;

    boost::asio::io_context io_;
    boost::asio::serial_port serial_;

    std::vector<int> motor_ids_;
    std::vector<std::string> joint_names_;
    std::map<int, MotorConfig> configs_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
    rclcpp::TimerBase::SharedPtr read_timer_;
};

}  // namespace so101_hardware

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<so101_hardware::So101HardwareNode>());
    rclcpp::shutdown();
    return 0;
}
