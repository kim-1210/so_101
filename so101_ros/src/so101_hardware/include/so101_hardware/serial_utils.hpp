#pragma once

#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <termios.h>

namespace so101_hardware
{

// 시리얼 설정 
inline void configure_serial(
    boost::asio::serial_port &serial,
    const std::string &port_name,
    unsigned int baudrate)
{
    serial.open(port_name);
    serial.set_option(boost::asio::serial_port_base::baud_rate(baudrate)); // 통신속도 (1초에 N비트 전송)
    serial.set_option(boost::asio::serial_port_base::character_size(8)); // 1개의 데이터당 8bit
    serial.set_option(boost::asio::serial_port_base::parity( // 패러티 비트 검사 
        boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits( // 정지신호 (끝 지점)
        boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::flow_control( // 패킷 받았다는 여부 확인
        boost::asio::serial_port_base::flow_control::none));
}

inline bool read_bytes(
    boost::asio::serial_port &serial,
    std::vector<uint8_t> &response,
    size_t expected_size)
{
    response.resize(expected_size);

    boost::system::error_code error;
    size_t bytes_read = boost::asio::read(
        serial,
        boost::asio::buffer(response),
        boost::asio::transfer_exactly(expected_size),
        error
    );

    if (error) {
        return false;
    }

    return bytes_read == expected_size;
}

inline void write_packet(
    boost::asio::serial_port &serial,
    const std::vector<uint8_t> &packet,
    int wait_ms = 20) // 20ms 기다림
{
    tcflush(serial.native_handle(), TCIFLUSH);
    boost::asio::write(serial, boost::asio::buffer(packet));
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
}

}  // namespace so101_hardware
