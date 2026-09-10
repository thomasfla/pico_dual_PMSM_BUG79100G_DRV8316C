#include "firmware.h"
#include <tusb.h>
#include "CoreMutex.h"
#include "USB.h"
#include <cstring>

static uint16_t telemetrySequence = 0;
bool usbPendingByteValid = false;
uint8_t usbPendingByte = 0;

struct __attribute__((packed)) UsbStatePacket {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t type;
  uint8_t version;
  uint8_t length;
  uint16_t sequence;
  uint32_t t_us;
  uint32_t latest_command_index;
  float control_loop_us;
  float m0_q;
  float m0_v;
  float m0_v_highfrequency;
  float m0_i;
  float m0_i_target;
  float m1_q;
  float m1_v;
  float m1_v_highfrequency;
  float m1_i;
  float m1_i_target;
  uint8_t flags;
  uint8_t checksum;
};

struct __attribute__((packed)) UsbCommandPacket {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t type;
  uint8_t version;
  uint8_t length;
  uint32_t command_index;
  uint8_t flags;
  uint16_t timeout_ms;
  float m0_kp;
  float m0_kd;
  float m0_iff;
  float m0_q_target;
  float m0_v_target;
  float m1_kp;
  float m1_kd;
  float m1_iff;
  float m1_q_target;
  float m1_v_target;
  uint8_t checksum;
};

static_assert(sizeof(UsbStatePacket) == 61, "Unexpected USB state packet size");
static_assert(sizeof(UsbCommandPacket) == 53, "Unexpected USB command packet size");
static constexpr uint16_t USB_COMMAND_RX_BYTE_BUDGET = sizeof(UsbCommandPacket) * 4u;
static constexpr uint32_t USB_COMMAND_RX_POLL_INTERVAL_US = 200;

// Latest-value mailboxes use an odd/even sequence counter so readers never see torn structs.
static volatile uint32_t runtimeStateSequence = 0;
static RuntimeControlState sharedRuntimeState;
static volatile uint32_t controlReferenceSequence = 0;
static ControlReferenceCommand sharedControlReference;

static void sharedMemoryBarrier() {
  __asm__ volatile("dmb sy" ::: "memory");
}

void publishRuntimeControlState(const RuntimeControlState &state) {
  uint32_t sequence = runtimeStateSequence;
  if ((sequence & 1u) != 0) {
    sequence++;
  }

  runtimeStateSequence = sequence + 1u;
  sharedMemoryBarrier();
  sharedRuntimeState = state;
  sharedMemoryBarrier();
  runtimeStateSequence = sequence + 2u;
}

bool readLatestRuntimeControlState(RuntimeControlState &state) {
  uint32_t sequenceBefore;
  uint32_t sequenceAfter;

  do {
    sequenceBefore = runtimeStateSequence;
    if (sequenceBefore == 0 || (sequenceBefore & 1u) != 0) {
      return false;
    }
    sharedMemoryBarrier();
    state = sharedRuntimeState;
    sharedMemoryBarrier();
    sequenceAfter = runtimeStateSequence;
  } while (sequenceBefore != sequenceAfter || (sequenceAfter & 1u) != 0);

  return true;
}

static void publishControlReferenceCommand(const ControlReferenceCommand &command) {
  uint32_t sequence = controlReferenceSequence;
  if ((sequence & 1u) != 0) {
    sequence++;
  }

  controlReferenceSequence = sequence + 1u;
  sharedMemoryBarrier();
  sharedControlReference = command;
  sharedMemoryBarrier();
  controlReferenceSequence = sequence + 2u;
}

bool readLatestControlReferenceCommand(
  ControlReferenceCommand &command,
  uint32_t &sequence
) {
  uint32_t sequenceBefore;
  uint32_t sequenceAfter;

  do {
    sequenceBefore = controlReferenceSequence;
    if (sequenceBefore == sequence || sequenceBefore == 0 || (sequenceBefore & 1u) != 0) {
      return false;
    }
    sharedMemoryBarrier();
    command = sharedControlReference;
    sharedMemoryBarrier();
    sequenceAfter = controlReferenceSequence;
  } while (sequenceBefore != sequenceAfter || (sequenceAfter & 1u) != 0);

  sequence = sequenceAfter;
  return true;
}

void discardControlReferenceCommands(uint32_t &sequence) {
  // If core0 is publishing, skip that in-flight command as well.
  const uint32_t current = controlReferenceSequence;
  sequence = (current + 1u) & ~1u;
  sharedMemoryBarrier();
}

static uint8_t packetChecksum(const uint8_t *bytes, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < (length - 1); i++) {
    checksum ^= bytes[i];
  }
  return checksum;
}

static uint8_t statePacketChecksum(const UsbStatePacket &packet) {
  const uint8_t *bytes = (const uint8_t *)&packet;
  return packetChecksum(bytes, sizeof(UsbStatePacket));
}

static bool commandPacketChecksumOk(const UsbCommandPacket &packet) {
  const uint8_t *bytes = (const uint8_t *)&packet;
  return packetChecksum(bytes, sizeof(UsbCommandPacket)) == packet.checksum;
}

static UsbStatePacket makeUsbStatePacket(const RuntimeControlState &state) {
  UsbStatePacket packet = {};
  packet.magic0 = USB_PACKET_MAGIC0;
  packet.magic1 = USB_PACKET_MAGIC1;
  packet.type = USB_PACKET_TYPE_STATE;
  packet.version = USB_PACKET_VERSION;
  packet.length = sizeof(UsbStatePacket);
  packet.sequence = telemetrySequence++;
  packet.t_us = state.tUs;
  packet.latest_command_index = state.latestCommandIndex;
  packet.control_loop_us = state.controlLoopUs;

  packet.m0_q = state.m0.position;
  packet.m0_v = state.m0.velocity;
  packet.m0_v_highfrequency = state.m0.velocityHighFrequency;
  packet.m0_i = state.m0.iq;
  packet.m0_i_target = state.m0.iqTarget;

  packet.m1_q = state.m1.position;
  packet.m1_v = state.m1.velocity;
  packet.m1_v_highfrequency = state.m1.velocityHighFrequency;
  packet.m1_i = state.m1.iq;
  packet.m1_i_target = state.m1.iqTarget;

  packet.flags = state.flags;
  packet.checksum = statePacketChecksum(packet);

  return packet;
}

void writeUsbStatePacketIfDue(const RuntimeControlState &state) {
  static uint32_t lastStatePacketUs = 0;

  if (state.tUs == lastStatePacketUs) {
    return;
  }
  if (lastStatePacketUs != 0 &&
      (state.tUs - lastStatePacketUs) < USB_STATE_FRAME_INTERVAL_US) {
    return;
  }

  CoreMutex usbLock(&USB.mutex, false);
  if (!usbLock) {
    return;
  }

  tud_task();
  if (!tud_cdc_connected()) {
    return;
  }
  if (tud_cdc_write_available() < sizeof(UsbStatePacket)) {
    lastStatePacketUs = state.tUs;
    return;
  }

  const UsbStatePacket packet = makeUsbStatePacket(state);
  tud_cdc_write((const uint8_t *)&packet, sizeof(packet));
  tud_task();
  tud_cdc_write_flush();
  lastStatePacketUs = state.tUs;
}

static bool commandPacketHeaderOk(const UsbCommandPacket &packet) {
  return packet.magic0 == USB_PACKET_MAGIC0 &&
    packet.magic1 == USB_PACKET_MAGIC1 &&
    packet.type == USB_PACKET_TYPE_COMMAND &&
    packet.version == USB_PACKET_VERSION &&
    packet.length == sizeof(UsbCommandPacket);
}

static MotorReferenceCommand makeMotorReferenceCommand(
  float kp,
  float kd,
  float iff,
  float qTarget,
  float vTarget
) {
  MotorReferenceCommand command;
  command.kp = kp;
  command.kd = kd;
  command.feedforwardCurrent = iff;
  command.targetPosition = qTarget;
  command.targetVelocity = vTarget;
  return command;
}

static void publishUsbCommandPacket(const UsbCommandPacket &packet, uint32_t nowUs) {
  ControlReferenceCommand command;
  command.tUs = nowUs;
  command.commandIndex = packet.command_index;
  command.timeoutMs = packet.timeout_ms;
  command.flags = packet.flags & (CONTROL_REFERENCE_FLAG_M0 | CONTROL_REFERENCE_FLAG_M1);
  command.m0 = makeMotorReferenceCommand(
    packet.m0_kp,
    packet.m0_kd,
    packet.m0_iff,
    packet.m0_q_target,
    packet.m0_v_target
  );
  command.m1 = makeMotorReferenceCommand(
    packet.m1_kp,
    packet.m1_kd,
    packet.m1_iff,
    packet.m1_q_target,
    packet.m1_v_target
  );

  publishControlReferenceCommand(command);
}

static void statusLedNoteCommandPacket(const UsbCommandPacket &packet, uint32_t nowUs) {
  const bool anyMotorCommand =
    (packet.flags & (CONTROL_REFERENCE_FLAG_M0 | CONTROL_REFERENCE_FLAG_M1)) != 0;
  statusLed.noteCommand(nowUs, packet.timeout_ms, anyMotorCommand);
}

static uint16_t readUsbCommandBytes(uint8_t *bytes, uint16_t capacity) {
  uint16_t count = 0;
  if (capacity == 0) {
    return count;
  }

  if (usbPendingByteValid) {
    bytes[count++] = usbPendingByte;
    usbPendingByteValid = false;
    if (count >= capacity) {
      return count;
    }
  }

  CoreMutex usbLock(&USB.mutex, false);
  if (!usbLock) {
    return count;
  }

  tud_task();
  const uint32_t available = tud_cdc_available();
  const uint32_t room = (uint32_t)(capacity - count);
  const uint32_t toRead = available < room ? available : room;
  if (toRead > 0) {
    count += (uint16_t)tud_cdc_read(bytes + count, toRead);
  }

  return count;
}

void readUsbCommandPackets() {
  static uint8_t rx[sizeof(UsbCommandPacket)];
  static uint8_t rxCount = 0;
  static uint32_t lastRxPollUs = 0;
  const uint32_t nowUs = micros();

  if (!usbPendingByteValid &&
      lastRxPollUs != 0 &&
      (nowUs - lastRxPollUs) < USB_COMMAND_RX_POLL_INTERVAL_US) {
    return;
  }
  lastRxPollUs = nowUs;

  uint8_t bytes[USB_COMMAND_RX_BYTE_BUDGET];
  const uint16_t bytesRead = readUsbCommandBytes(bytes, sizeof(bytes));

  for (uint16_t i = 0; i < bytesRead; i++) {
    const uint8_t byte = bytes[i];

    if (rxCount == 0) {
      if (byte == USB_PACKET_MAGIC0) {
        rx[rxCount++] = byte;
      }
      continue;
    }

    if (rxCount == 1) {
      if (byte == USB_PACKET_MAGIC1) {
        rx[rxCount++] = byte;
      } else if (byte != USB_PACKET_MAGIC0) {
        rxCount = 0;
      }
      continue;
    }

    rx[rxCount++] = byte;

    if (rxCount == 5 && rx[4] != sizeof(UsbCommandPacket)) {
      rxCount = (byte == USB_PACKET_MAGIC0) ? 1 : 0;
      if (rxCount == 1) {
        rx[0] = byte;
      }
      continue;
    }

    if (rxCount < sizeof(UsbCommandPacket)) {
      continue;
    }

    UsbCommandPacket packet;
    memcpy(&packet, rx, sizeof(packet));
    rxCount = 0;

    if (commandPacketHeaderOk(packet) && commandPacketChecksumOk(packet)) {
      const uint32_t nowUs = micros();
      statusLedNoteCommandPacket(packet, nowUs);
      publishUsbCommandPacket(packet, nowUs);
    }
  }
}
