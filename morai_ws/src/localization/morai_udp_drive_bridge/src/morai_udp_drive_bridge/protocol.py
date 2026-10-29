"""MORAI Competition-only status input and EgoCtrlCmd output.

공식 24.R2 계열 구조체의 ``_pack_ = 1`` little-endian 레이아웃을
Python struct로 명시한다. 기본/확장 상태 수신은 181/229바이트, 제어 송신은
55바이트이다. Competition 입력은 아래의 별도 제한 필드 parser로
팀에서 확인한 181/229바이트 형식을 검증하며 예약된 truth 슬롯은 사용하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


COMPETITION_STATUS_LAYOUTS = {181: 152, 229: 200}
STATUS_HEADER = b"#MoraiInfo$"
EGO_CTRL_CMD_FORMAT = "<14s i 3i 3b 5f 2s"
EGO_CTRL_CMD_PACKET_SIZE = struct.calcsize(EGO_CTRL_CMD_FORMAT)


class ProtocolError(ValueError):
    """MORAI Competition 입력 또는 제어 패킷의 프로토콜 오류."""


@dataclass(frozen=True)
class CompetitionVehicleStatusMeasurement:
    """Only the permitted Competition fields; unavailable truth is not decoded.

    Layout evidence: the team's path_planning/morai_udp_competition_status.py
    (181/229 bytes), corroborated by the shipped NetworkModule packed struct.
    Both layouts have the same offsets through steer. The 229-byte tire
    extension and reserved truth slots are deliberately ignored.
    """
    sec: int
    nsec: int
    ctrl_mode: int
    gear: int
    signed_velocity_kmh: float
    accel_pedal: float
    brake_pedal: float
    size_x_m: float
    size_y_m: float
    size_z_m: float
    overhang_m: float
    wheelbase_m: float
    rear_overhang_m: float
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    velocity_x_kmh: float
    angular_velocity_x_deg_s: float
    angular_velocity_y_deg_s: float
    angular_velocity_z_deg_s: float
    steer_deg: float
    packet_length: int


def parse_competition_vehicle_status(packet: bytes) -> CompetitionVehicleStatusMeasurement:
    """Validate a complete datagram; never truncate an extended datagram."""
    expected_length = COMPETITION_STATUS_LAYOUTS.get(len(packet))
    if expected_length is None:
        raise ProtocolError(f"Competition status requires 181 or 229 bytes, got {len(packet)}")
    if packet[:11] != STATUS_HEADER:
        raise ProtocolError(f"Competition status invalid header: {packet[:11]!r}")
    if struct.unpack_from('<I', packet, 11)[0] != expected_length:
        raise ProtocolError('Competition status invalid declared data length')
    if packet[-2:] != b'\r\n':
        raise ProtocolError('Competition status invalid tail')
    sec, nsec = struct.unpack_from('<II', packet, 27)
    if nsec >= 1_000_000_000:
        raise ProtocolError('Competition status invalid nanoseconds')
    ctrl_mode, gear = struct.unpack_from('<bb', packet, 35)
    def scalar(offset):
        return _finite(struct.unpack_from('<f', packet, offset)[0], f'offset_{offset}')
    return CompetitionVehicleStatusMeasurement(
        sec, nsec, ctrl_mode, gear, scalar(37), scalar(45), scalar(49),
        scalar(53), scalar(57), scalar(61), scalar(65), scalar(69), scalar(73),
        scalar(89), scalar(93), scalar(97), scalar(101),
        scalar(113), scalar(117), scalar(121), scalar(137), len(packet))


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ProtocolError(f"CompetitionVehicleStatus {name} 값이 유한하지 않다: {value}")
    return value


def build_ego_ctrl_cmd(
    *,
    cmd_type: int,
    velocity_kmh: float = 0.0,
    acceleration_mps2: float = 0.0,
    accel: float = 0.0,
    brake: float = 0.0,
    steer_normalized: float = 0.0,
    ctrl_mode: int = 2,
    gear: int = 4,
) -> bytes:
    """공식 EgoCtrlCmd 55바이트 패킷을 생성한다.

    ``steer_normalized``는 실제 앞바퀴각/최대앞바퀴각이며 -1~1이다.
    ``velocity_kmh``는 MORAI UDP 프로토콜 단위인 km/h이다.
    """

    values = (
        b"#MoraiCtrlCmd$",
        23,
        0,
        0,
        0,
        int(ctrl_mode),
        int(gear),
        int(cmd_type),
        float(velocity_kmh),
        float(acceleration_mps2),
        max(0.0, min(1.0, float(accel))),
        max(0.0, min(1.0, float(brake))),
        max(-1.0, min(1.0, float(steer_normalized))),
        b"\r\n",
    )
    packet = struct.pack(EGO_CTRL_CMD_FORMAT, *values)
    if len(packet) != EGO_CTRL_CMD_PACKET_SIZE:
        raise ProtocolError(f"EgoCtrlCmd 생성 길이 오류: {len(packet)}")
    return packet
