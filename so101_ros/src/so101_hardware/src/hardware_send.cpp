// ros2 run so101_hardware hardware_send -- leader -1.5 -2.7 2.6 1.6 -1.5 -2.7 300 20 300
// ros2 run so101_hardware hardware_send -- leader -1.9 2.59 -2.5 1.3 -1.8 -2.58 300 20 300

#include <boost/asio.hpp>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "so101_hardware/common.hpp"
#include "so101_hardware/packet.hpp"
#include "so101_hardware/serial_utils.hpp"

using namespace so101_hardware;

int main(int argc, char *argv[])
{
    if (argc < 8) {
        std::cerr << "Usage: hardware_send <leader|follower> <rad1> <rad2> <rad3> <rad4> <rad5> <rad6> [speed] [acc] [torque_limit] [port]" << std::endl;
        return 1;
    }

    std::string arm_type = argv[1];
    std::vector<double> target_rads = {
        std::stod(argv[2]),
        std::stod(argv[3]),
        std::stod(argv[4]),
        std::stod(argv[5]),
        std::stod(argv[6]),
        std::stod(argv[7])
    };

    uint16_t speed = argc >= 9 ? clamp_u16(std::stoi(argv[8]), 0, 4095) : 1200;
    uint8_t acc = argc >= 10 ? static_cast<uint8_t>(clamp_u16(std::stoi(argv[9]), 0, 255)) : 50;
    uint16_t torque_limit = argc >= 11 ? clamp_u16(std::stoi(argv[10]), 0, 1000) : 1000;
    std::string port_name = argc >= 12 ? argv[11] : DEFAULT_PORT;

    try {
        auto motor_ids = motor_ids_for_arm(arm_type);
        auto configs = load_motor_config(default_config_path_for_arm(arm_type));

        boost::asio::io_context io;
        boost::asio::serial_port serial(io);
        configure_serial(serial, port_name, DEFAULT_BAUDRATE);

        std::cout << "Serial opened: " << port_name << std::endl;
        std::cout << "speed: " << speed
                  << " acc: " << static_cast<int>(acc)
                  << " torque_limit: " << torque_limit << std::endl;

        for (size_t index = 0; index < motor_ids.size(); ++index) {
            int motor_id = motor_ids[index];
            uint8_t id = static_cast<uint8_t>(motor_id);

            int config_id = config_id_for_motor(arm_type, motor_id);
            const auto &config = configs.at(config_id);

            uint16_t raw_goal = calibrated_rad_to_tick(target_rads[index], config);

            std::cout << "Sending ID " << motor_id
                      << " target_rad: " << target_rads[index]
                      << " calibrated_goal_tick: " << raw_goal
                      << " raw_goal: " << raw_goal
                      << std::endl;

            write_packet(serial, make_torque_enable_packet(id, true));
            write_packet(serial, make_torque_limit_packet(id, torque_limit));
            write_packet(serial, make_goal_packet(id, raw_goal, speed, acc));

            std::cout << "ID " << motor_id << " command sent" << std::endl;
        }

        serial.close();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
