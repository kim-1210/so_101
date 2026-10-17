#pragma once

#include <cstdint>
#include <vector>

namespace so101_hardware
{

constexpr uint8_t INST_READ = 0x02;
constexpr uint8_t INST_WRITE = 0x03;

constexpr uint8_t ADDR_TORQUE_ENABLE = 40; // 토크 실행 여부
constexpr uint8_t ADDR_ACC = 41;
constexpr uint8_t ADDR_PRESENT_POSITION = 56; // 상태 가져오기
constexpr uint8_t ADDR_TORQUE_LIMIT = 48; // 토크 최대 설정
constexpr uint8_t READ_FEEDBACK_SIZE = 6;

// checksum 제작
inline uint8_t checksum(const std::vector<uint8_t> &packet)
{
    uint16_t sum = 0;

    for (size_t i = 2; i < packet.size() - 1; ++i) {
        sum += packet[i];
    }

    return static_cast<uint8_t>(~sum);
}

// checksum 비교
inline bool check_response_checksum(const std::vector<uint8_t> &response)
{
    return !response.empty() && checksum(response) == response.back();
}

// 모터 상태확인 패킷
inline std::vector<uint8_t> make_read_feedback_packet(uint8_t id)
{
    std::vector<uint8_t> packet = {
        0xFF,
        0xFF,
        id,
        0x04,
        INST_READ,
        ADDR_PRESENT_POSITION,
        READ_FEEDBACK_SIZE,
        0x00
    };

    packet.back() = checksum(packet);
    return packet;
}

// 모터 각도 보내기
inline std::vector<uint8_t> make_write_packet(
    uint8_t id,
    uint8_t address,
    const std::vector<uint8_t> &data)
{
    std::vector<uint8_t> packet = {
        0xFF,
        0xFF,
        id,
        static_cast<uint8_t>(data.size() + 3),
        INST_WRITE,
        address
    };

    packet.insert(packet.end(), data.begin(), data.end());
    packet.push_back(0x00);
    packet.back() = checksum(packet);

    return packet;
}

// 토크 실행 여부
inline std::vector<uint8_t> make_torque_enable_packet(uint8_t id, bool enable)
{
    return make_write_packet(id, ADDR_TORQUE_ENABLE, {static_cast<uint8_t>(enable ? 1 : 0)});
}

// 최대 토크 설정
inline std::vector<uint8_t> make_torque_limit_packet(uint8_t id, uint16_t torque_limit)
{
    uint8_t torque_l = torque_limit & 0xFF;
    uint8_t torque_h = (torque_limit >> 8) & 0xFF;

    return make_write_packet(id, ADDR_TORQUE_LIMIT, {torque_l, torque_h});
}

// 각도 설정 패킷
inline std::vector<uint8_t> make_goal_packet(
    uint8_t id,
    uint16_t tick,
    uint16_t speed,
    uint8_t acc)
{
    uint8_t pos_l = tick & 0xFF;
    uint8_t pos_h = (tick >> 8) & 0xFF;
    uint8_t speed_l = speed & 0xFF;
    uint8_t speed_h = (speed >> 8) & 0xFF;

    return make_write_packet(id, ADDR_ACC, {
        acc,
        pos_l,
        pos_h,
        0x00,
        0x00,
        speed_l,
        speed_h
    });
}

}  // namespace so101_hardware
