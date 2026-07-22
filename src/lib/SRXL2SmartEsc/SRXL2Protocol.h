#pragma once

#include <stddef.h>
#include <stdint.h>

namespace srxl2smart
{
constexpr uint8_t MAGIC = 0xA6;
constexpr uint8_t PACKET_HANDSHAKE = 0x21;
constexpr uint8_t PACKET_TELEMETRY = 0x80;
constexpr uint8_t PACKET_CONTROL = 0xCD;
constexpr uint8_t COMMAND_CHANNEL = 0x00;
constexpr uint8_t COMMAND_CHANNEL_FAILSAFE = 0x01;
constexpr uint8_t MASTER_DEVICE_ID = 0x10;
constexpr uint8_t ESC_DEVICE_ID_MIN = 0x40;
constexpr uint8_t ESC_DEVICE_ID_MAX = 0x43;
constexpr size_t MAX_PACKET_SIZE = 80;
constexpr size_t TELEMETRY_PAYLOAD_SIZE = 16;

struct SmartEscTelemetry
{
    bool rpmValid = false;
    bool voltageValid = false;
    bool currentValid = false;
    bool capacityValid = false;
    bool fetTemperatureValid = false;
    bool becTemperatureValid = false;

    uint32_t rpm = 0;
    uint16_t voltageDeciVolts = 0;
    uint16_t currentDeciAmps = 0;
    uint32_t capacityMah = 0;
    int16_t fetTemperatureDeciC = 0;
    int16_t becTemperatureDeciC = 0;
};

uint16_t crc16(const uint8_t *data, size_t length);
bool validateFrame(const uint8_t *frame, size_t length);

size_t buildHandshake(uint8_t *frame, uint8_t sourceId, uint8_t destinationId,
                      uint8_t priority, uint8_t baudSupported, uint8_t deviceInfo,
                      uint32_t uid);
size_t buildChannel(uint8_t *frame, uint8_t replyId, uint16_t throttle,
                    bool failsafe, int8_t rssi = 0, uint16_t frameLosses = 0);

bool decodeTelemetryPayload(const uint8_t payload[TELEMETRY_PAYLOAD_SIZE],
                            SmartEscTelemetry &telemetry);
} // namespace srxl2smart
