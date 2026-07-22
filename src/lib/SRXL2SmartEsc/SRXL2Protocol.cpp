#include "SRXL2Protocol.h"

namespace srxl2smart
{
namespace
{
inline uint16_t readBe16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

inline int16_t readBeI16(const uint8_t *data)
{
    return static_cast<int16_t>(readBe16(data));
}

inline void writeLe16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
}

inline void writeLe32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
    data[2] = static_cast<uint8_t>(value >> 16);
    data[3] = static_cast<uint8_t>(value >> 24);
}

inline void appendCrc(uint8_t *frame, size_t length)
{
    const uint16_t crc = crc16(frame, length - 2);
    frame[length - 2] = static_cast<uint8_t>(crc >> 8);
    frame[length - 1] = static_cast<uint8_t>(crc);
}
} // namespace

uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0;
    for (size_t index = 0; index < length; ++index)
    {
        crc ^= static_cast<uint16_t>(data[index]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool validateFrame(const uint8_t *frame, size_t length)
{
    if (frame == nullptr || length < 5 || length > MAX_PACKET_SIZE ||
        frame[0] != MAGIC || frame[2] != length)
    {
        return false;
    }

    const uint16_t expected = readBe16(&frame[length - 2]);
    return crc16(frame, length - 2) == expected;
}

size_t buildHandshake(uint8_t *frame, uint8_t sourceId, uint8_t destinationId,
                      uint8_t priority, uint8_t baudSupported, uint8_t deviceInfo,
                      uint32_t uid)
{
    constexpr size_t length = 14;
    frame[0] = MAGIC;
    frame[1] = PACKET_HANDSHAKE;
    frame[2] = length;
    frame[3] = sourceId;
    frame[4] = destinationId;
    frame[5] = priority;
    frame[6] = baudSupported;
    frame[7] = deviceInfo;
    writeLe32(&frame[8], uid);
    appendCrc(frame, length);
    return length;
}

size_t buildChannel(uint8_t *frame, uint8_t replyId, uint16_t throttle,
                    bool failsafe, int8_t rssi, uint16_t frameLosses)
{
    // Only SRXL2 channel 0 is required by an ESC. The selected ExpressLRS
    // input channel is deliberately remapped to this throttle channel.
    constexpr uint32_t channelMask = 0x00000001U;
    constexpr size_t length = 16;

    frame[0] = MAGIC;
    frame[1] = PACKET_CONTROL;
    frame[2] = length;
    frame[3] = failsafe ? COMMAND_CHANNEL_FAILSAFE : COMMAND_CHANNEL;
    frame[4] = failsafe ? 0 : replyId;
    frame[5] = static_cast<uint8_t>(rssi);
    writeLe16(&frame[6], frameLosses);
    writeLe32(&frame[8], channelMask);
    writeLe16(&frame[12], static_cast<uint16_t>(throttle & 0xFFFCU));
    appendCrc(frame, length);
    return length;
}

bool decodeTelemetryPayload(const uint8_t payload[TELEMETRY_PAYLOAD_SIZE],
                            SmartEscTelemetry &telemetry)
{
    if (payload == nullptr)
    {
        return false;
    }

    switch (payload[0])
    {
    case 0x20: // Spektrum ESC telemetry, big-endian
    {
        const uint16_t rawRpm = readBe16(&payload[2]);
        const uint16_t rawVoltage = readBe16(&payload[4]);
        const uint16_t rawFetTemperature = readBe16(&payload[6]);
        const uint16_t rawMotorCurrent = readBe16(&payload[8]);
        const uint16_t rawBecTemperature = readBe16(&payload[10]);

        if (rawRpm != 0xFFFFU)
        {
            telemetry.rpm = static_cast<uint32_t>(rawRpm) * 10U;
            telemetry.rpmValid = true;
        }
        if (rawVoltage != 0xFFFFU)
        {
            telemetry.voltageDeciVolts = static_cast<uint16_t>(rawVoltage / 10U);
            telemetry.voltageValid = true;
        }
        if (rawMotorCurrent != 0xFFFFU)
        {
            telemetry.currentDeciAmps = static_cast<uint16_t>(rawMotorCurrent / 10U);
            telemetry.currentValid = true;
        }
        if (rawFetTemperature != 0xFFFFU)
        {
            telemetry.fetTemperatureDeciC = static_cast<int16_t>(rawFetTemperature);
            telemetry.fetTemperatureValid = true;
        }
        if (rawBecTemperature != 0xFFFFU)
        {
            telemetry.becTemperatureDeciC = static_cast<int16_t>(rawBecTemperature);
            telemetry.becTemperatureValid = true;
        }
        return true;
    }

    case 0x34: // Flight-pack current/capacity, big-endian
    {
        const int16_t rawCurrent = readBeI16(&payload[2]);
        const int16_t rawCapacity = readBeI16(&payload[4]);
        const uint16_t rawTemperature = readBe16(&payload[6]);

        if (static_cast<uint16_t>(rawCurrent) != 0x7FFFU && rawCurrent >= 0)
        {
            telemetry.currentDeciAmps = static_cast<uint16_t>(rawCurrent);
            telemetry.currentValid = true;
        }
        if (static_cast<uint16_t>(rawCapacity) != 0x7FFFU && rawCapacity >= 0)
        {
            telemetry.capacityMah = static_cast<uint16_t>(rawCapacity);
            telemetry.capacityValid = true;
        }
        if (rawTemperature != 0x7FFFU && rawTemperature != 0xFFFFU)
        {
            telemetry.fetTemperatureDeciC = static_cast<int16_t>(rawTemperature);
            telemetry.fetTemperatureValid = true;
        }
        return true;
    }

    default:
        // SMART Battery (0x42) is intentionally not decoded yet. Published
        // implementations disagree about its byte order and scaling, so it
        // remains feature-gated until an Avian bus capture verifies it.
        return false;
    }
}
} // namespace srxl2smart
