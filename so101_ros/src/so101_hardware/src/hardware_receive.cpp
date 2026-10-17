// ros2 run so101_hardware hardware_receive -- follower

#include <boost/asio.hpp>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "so101_hardware/common.hpp"
#include "so101_hardware/packet.hpp"
#include "so101_hardware/serial_utils.hpp"

using namespace so101_hardware;

bool validate_feedback_response(
    const std::vector<uint8_t> &response,
    uint8_t expected_id,
    int motor_id)
{
    if (response[0] != 0xFF || response[1] != 0xFF) {
        std::cout << "ID " << motor_id << ": invalid header" << std::endl;
        return false;
    }

    if (response[2] != expected_id) {
        std::cout << "ID " << motor_id << ": wrong response id" << std::endl;
        return false;
    }

    if (response[4] != 0) {
        std::cout << "ID " << motor_id << ": motor error " << static_cast<int>(response[4]) << std::endl;
        return false;
    }

    if (!check_response_checksum(response)) {
        std::cout << "ID " << motor_id << ": checksum error" << std::endl;
        return false;
    }

    return true;
}

void print_feedback(
    const std::string &arm_type,
    const std::map<int, MotorConfig> &configs,
    int motor_id,
    const std::vector<uint8_t> &response)
{
    int position = response[5] | (response[6] << 8);
    uint16_t speed_raw = response[7] | (response[8] << 8);
    uint16_t load_raw = response[9] | (response[10] << 8);
    int16_t speed = decode_sign_magnitude_15(speed_raw);
    int16_t load = decode_sign_magnitude_15(load_raw);

    int config_id = config_id_for_motor(arm_type, motor_id);
    const auto &config = configs.at(config_id);

    std::cout << "ID " << motor_id
              << " raw: " << position
              << " calibrated_mid: " << calibration_mid_tick(config)
              << " rad: " << calibrated_tick_to_rad(position, config)
              << " degree: " << calibrated_tick_to_degree(position, config)
              << " speed_raw: " << speed_raw
              << " speed: " << speed
              << " load_raw: " << load_raw
              << " load: " << load
              << std::endl;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: hardware_receive <leader|follower> [port]" << std::endl;
        return 1;
    }

    std::string arm_type = argv[1];
    std::string port_name = argc >= 3 ? argv[2] : DEFAULT_PORT;

    try {
        auto motor_ids = motor_ids_for_arm(arm_type);
        auto configs = load_motor_config(default_config_path_for_arm(arm_type));

        boost::asio::io_context io;
        boost::asio::serial_port serial(io);
        configure_serial(serial, port_name, DEFAULT_BAUDRATE);

        std::cout << "Serial opened: " << port_name << std::endl;

        for (int motor_id : motor_ids) {
            uint8_t id = static_cast<uint8_t>(motor_id);

            write_packet(serial, make_read_feedback_packet(id), 0);

            std::vector<uint8_t> response;
            bool ok = read_bytes(serial, response, READ_FEEDBACK_SIZE + 6);

            if (!ok) {
                std::cout << "ID " << motor_id << ": no response" << std::endl;
                continue;
            }

            if (!validate_feedback_response(response, id, motor_id)) {
                continue;
            }

            print_feedback(arm_type, configs, motor_id, response);
        }

        serial.close();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
