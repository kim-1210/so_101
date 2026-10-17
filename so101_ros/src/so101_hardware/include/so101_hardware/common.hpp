#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace so101_hardware
{

constexpr const char * LEADER_CONFIG_PATH =
    "/media/kim/hdd1tb/master_study/so_101/so101_ros/src/so101_hardware/config/so101_leader.json";
constexpr const char * FOLLOWER_CONFIG_PATH =
    "/media/kim/hdd1tb/master_study/so_101/so101_ros/src/so101_hardware/config/so101_follower.json";
constexpr const char * DEFAULT_CONFIG_PATH = LEADER_CONFIG_PATH;
constexpr const char * DEFAULT_PORT = "/dev/ttyACM0";
constexpr unsigned int DEFAULT_BAUDRATE = 1000000;
constexpr int TICKS_PER_REVOLUTION = 4096;

struct MotorConfig {
    int id;
    int homing_offset;
    int range_min;
    int range_max;
    double zero_offset_rad;
};

inline std::map<int, MotorConfig> load_motor_config(const std::string &path)
{
    std::ifstream file(path);

    if (!file.is_open()) {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    nlohmann::json data;
    file >> data;

    std::map<int, MotorConfig> configs;

    for (auto &[name, value] : data.items()) {
        (void)name;

        MotorConfig config;
        config.id = value["id"];
        config.homing_offset = value["homing_offset"];
        config.range_min = value["range_min"];
        config.range_max = value["range_max"];
        config.zero_offset_rad = value.value("zero_offset_rad", 0.0);

        configs[config.id] = config;
    }

    return configs;
}

inline std::vector<int> motor_ids_for_arm(const std::string &arm_type)
{
    if (arm_type == "leader") {
        return {1, 2, 3, 4, 5, 6};
    }

    if (arm_type == "follower") {
        return {11, 12, 13, 14, 15, 16};
    }

    throw std::runtime_error("arm_type must be leader or follower");
}

inline int config_id_for_motor(const std::string &arm_type, int motor_id)
{
    (void)arm_type;
    return motor_id;
}

inline std::string default_config_path_for_arm(const std::string &arm_type)
{
    if (arm_type == "leader") {
        return LEADER_CONFIG_PATH;
    }

    if (arm_type == "follower") {
        return FOLLOWER_CONFIG_PATH;
    }

    throw std::runtime_error("arm_type must be leader or follower");
}

inline uint16_t clamp_u16(int value, int min_value, int max_value)
{
    return static_cast<uint16_t>(std::clamp(value, min_value, max_value));
}

inline uint16_t normalize_tick(int tick)
{
    while (tick < 0) {
        tick += TICKS_PER_REVOLUTION;
    }

    while (tick >= TICKS_PER_REVOLUTION) {
        tick -= TICKS_PER_REVOLUTION;
    }

    return static_cast<uint16_t>(tick);
}

inline uint16_t rad_to_tick(double rad)
{
    int tick = static_cast<int>((rad + M_PI) * TICKS_PER_REVOLUTION / (2.0 * M_PI));
    return normalize_tick(tick);
}

inline double corrected_tick_to_rad(int corrected_tick)
{
    return (corrected_tick - 2048) * 2.0 * M_PI / TICKS_PER_REVOLUTION;
}

inline double corrected_tick_to_degree(int corrected_tick)
{
    return (corrected_tick - 2048) * 360.0 / TICKS_PER_REVOLUTION;
}

inline double calibration_mid_tick(const MotorConfig &config)
{
    return (static_cast<double>(config.range_min) + static_cast<double>(config.range_max)) / 2.0;
}

inline uint16_t calibrated_rad_to_tick(double rad, const MotorConfig &config)
{
    const double motor_rad = rad + config.zero_offset_rad;
    const double tick = (motor_rad * (TICKS_PER_REVOLUTION - 1) / (2.0 * M_PI)) +
                        calibration_mid_tick(config);
    const int rounded_tick = static_cast<int>(std::lround(tick));
    return clamp_u16(rounded_tick, config.range_min, config.range_max);
}

inline double calibrated_tick_to_rad(int tick, const MotorConfig &config)
{
    const double motor_rad =
        (static_cast<double>(tick) - calibration_mid_tick(config)) * 2.0 * M_PI /
        (TICKS_PER_REVOLUTION - 1);

    return motor_rad - config.zero_offset_rad;
}

inline double calibrated_tick_to_degree(int tick, const MotorConfig &config)
{
    return (static_cast<double>(tick) - calibration_mid_tick(config)) * 360.0 /
           (TICKS_PER_REVOLUTION - 1);
}

inline int16_t decode_sign_magnitude_15(uint16_t raw)
{
    int16_t value = raw & 0x7FFF;

    if ((raw & 0x8000) != 0) {
        value = -value;
    }

    return value;
}

}  // namespace so101_hardware
