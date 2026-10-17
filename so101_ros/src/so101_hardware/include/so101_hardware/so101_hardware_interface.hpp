#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "boost/asio.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "so101_hardware/common.hpp"

namespace so101_hardware {

// So101HardwareInterface가 어떤 함수를 쓸지, 어떻게 정의할지에 대한 껍데기 제작
class So101HardwareInterface:
public hardware_interface::SystemInterface{
    public:
        // constexpr : 컴파일 단계에서 확정 낼 수 있음
        static constexpr std::size_t kJointCount{6};

        // 상태값 vector로
        std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;
        // 명령값 vector로
        std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

        // hardware_interface의 초기상태
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info
        ) override;
        // 하드웨어 사용할 준비
        // (시리얼 오픈, 장치 탐색, 메모리 준비, 포트 연결)
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state
        ) override;
        // 하드웨어 제어 시작
        // (토크 실행, 엔코더 초기화, 명령 및 상태 동기화)
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state
        ) override;
        // 하드웨어 제어 종료
        // (토크 해제, 안전 정지)
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state
        ) override;

        // read = state
        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period
        ) override;
        // write = command
        hardware_interface::return_type write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period
        ) override;
    
    private:
        bool read_motor_feedback(
            uint8_t motor_id,
            double &position_rad,
            double &velocity_raw,
            double &effort_raw
        );

        std::string arm_type_;
        std::string port_name_;
        std::string config_path_;

        uint16_t speed_{300};
        uint8_t acc_{20};
        uint16_t torque_limit_{300};

        boost::asio::io_context io_;
        boost::asio::serial_port serial_{io_};

        std::array<int, kJointCount> motor_ids_{};
        std::map<int, MotorConfig> configs_;

        std::array<double, kJointCount> hw_positions_{};
        std::array<double, kJointCount> hw_velocities_{};
        std::array<double, kJointCount> hw_efforts_{};
        std::array<double, kJointCount> hw_commands_{};
};

} // namespace so101_hardware 등록
