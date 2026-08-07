import glob
import struct


MAGIC0 = 0xA5
MAGIC1 = 0x5A
MAGIC = bytes((MAGIC0, MAGIC1))
VERSION = 2
PACKET_TYPE_COMMAND = 0x43
PACKET_TYPE_STATE = 0x53

M0_READY = 1 << 0
M1_READY = 1 << 1
BOTH_MOTORS = M0_READY | M1_READY

COMMAND_FRAME = struct.Struct("<BBBBBIBH10fB")
STATE_FRAME = struct.Struct("<BBBBBHIIf10fBB")
STATE_VALUE_NAMES = (
    "m0_q",
    "m0_v",
    "m0_v_highfrequency",
    "m0_i",
    "m0_i_target",
    "m1_q",
    "m1_v",
    "m1_v_highfrequency",
    "m1_i",
    "m1_i_target",
)


def default_port():
    ports = sorted(
        glob.glob("/dev/serial/by-id/*")
        + glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyUSB*")
    )
    if not ports:
        raise SystemExit("No serial port found. Pass --port /dev/ttyACM0")
    return ports[0]


def checksum(frame_bytes):
    value = 0
    for byte in frame_bytes[:-1]:
        value ^= byte
    return value


def checksum_ok(frame_bytes):
    return checksum(frame_bytes) == frame_bytes[-1]


def encode_command(command_index, flags, timeout_ms, m0, m1):
    packet = COMMAND_FRAME.pack(
        MAGIC0,
        MAGIC1,
        PACKET_TYPE_COMMAND,
        VERSION,
        COMMAND_FRAME.size,
        command_index & 0xFFFFFFFF,
        flags & 0xFF,
        timeout_ms & 0xFFFF,
        *(tuple(m0) + tuple(m1)),
        0,
    )
    return packet[:-1] + bytes((checksum(packet),))


def decode_state(frame_bytes):
    if len(frame_bytes) != STATE_FRAME.size:
        return None
    if frame_bytes[0:2] != MAGIC:
        return None
    if frame_bytes[2] != PACKET_TYPE_STATE:
        return None
    if frame_bytes[3] != VERSION or frame_bytes[4] != STATE_FRAME.size:
        return None
    if not checksum_ok(frame_bytes):
        return None

    unpacked = STATE_FRAME.unpack(frame_bytes)
    values = dict(zip(STATE_VALUE_NAMES, unpacked[9:19]))
    values.update(
        sequence=unpacked[5],
        t_us=unpacked[6],
        latest_command_index=unpacked[7],
        control_loop_us=unpacked[8],
        flags=unpacked[19],
    )
    return values


def pop_state_frames(buffer):
    frames = []
    while len(buffer) >= STATE_FRAME.size:
        magic_at = buffer.find(MAGIC)
        if magic_at < 0:
            del buffer[:-1]
            break
        if magic_at:
            del buffer[:magic_at]
        if len(buffer) < 5:
            break
        packet_len = buffer[4]
        if (
            buffer[2] != PACKET_TYPE_STATE
            or buffer[3] != VERSION
            or packet_len != STATE_FRAME.size
        ):
            del buffer[0]
            continue
        if len(buffer) < packet_len:
            break

        candidate = bytes(buffer[:packet_len])
        del buffer[:packet_len]
        decoded = decode_state(candidate)
        if decoded is not None:
            frames.append(decoded)
    return frames
