# 등록시 모터는 반드시 1개씩 등록
from pathlib import Path

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ports = ["/dev/ttyACM0", "/dev/ttyUSB0"]
port = next((p for p in ports if Path(p).exists()), ports[0])
print(f"=========\n{port}\n=========\n")
baudrate = 1000000

portHandler = PortHandler(port)
packetHandler = PacketHandler(1)

if not portHandler.openPort():
    raise Exception("포트 열기 실패")

print("포트열림")

# baudrate 설정
if not portHandler.setBaudRate(baudrate):
    raise Exception("baudrate 설정 실패")

print("baudrate 설정 성공")

old_id = None

for servo_id in range(0, 21):

    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(
        portHandler,
        servo_id
    )

    if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
        old_id = servo_id
        print(f"서보 발견 ID: {servo_id}")

if old_id == None:
    raise Exception("등록 Id 없음")

new_id = int(input("등록할 new id: "))

SMS_STS_ID = 5 # ID등록 메모리 주소
SMS_STS_LOCK = 55 # EEPROM 잠금 주소


def check_write(result, error, action):
    if result != COMM_SUCCESS:
        raise Exception(f"{action} 실패: {packetHandler.getTxRxResult(result)}")
    if error != 0:
        raise Exception(f"{action} 실패: {packetHandler.getRxPacketError(error)}")


result, error = packetHandler.write1ByteTxRx(
    portHandler,
    old_id,
    SMS_STS_LOCK,
    0
)
check_write(result, error, "EEPROM 잠금 해제")

scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
    portHandler,
    old_id,
    SMS_STS_ID,
    new_id
)

if scs_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print(packetHandler.getRxPacketError(scs_error))
else:
    print(f"ID 변경 성공: {old_id} -> {new_id}")

# EEPROM lock
result, error = packetHandler.write1ByteTxRx(
    portHandler,
    new_id,
    SMS_STS_LOCK,
    1
)
check_write(result, error, "EEPROM 잠금")

portHandler.closePort()
