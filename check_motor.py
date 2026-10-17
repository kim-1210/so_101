from pathlib import Path

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PORTS = ["/dev/ttyACM1", "/dev/ttyACM2"]
BAUDRATE = 1000000
MIN_ID = 0
MAX_ID = 253


def main():
    existing_ports = [p for p in PORTS if Path(p).exists()]

    if not existing_ports:
        raise Exception(f"검사할 포트가 없습니다: {PORTS}")

    packet_handler = PacketHandler(1)

    for port in existing_ports:
        port_handler = PortHandler(port)

        if not port_handler.openPort():
            print(f"\n포트 열기 실패: {port}")
            continue

        print(f"\n포트열림: {port}")

        if not port_handler.setBaudRate(BAUDRATE):
            port_handler.closePort()
            print(f"baudrate 설정 실패: {BAUDRATE}")
            continue

        print(f"baudrate 설정 성공: {BAUDRATE}")
        print("모터 ID 검색중...")

        found_ids = []

        for servo_id in range(MIN_ID, MAX_ID + 1):
            model_number, comm_result, error = packet_handler.ping(
                port_handler,
                servo_id
            )

            if comm_result == COMM_SUCCESS and error == 0:
                found_ids.append(servo_id)
                print(f"서보 발견 ID: {servo_id}, model: {model_number}")

        port_handler.closePort()

        if found_ids:
            print(f"{port} 연결된 ID: {found_ids}")
        else:
            print(f"{port} 연결된 서보를 찾지 못했습니다.")


if __name__ == "__main__":
    main()
