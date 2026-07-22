#include <unity.h>

#include "SRXL2Protocol.h"

using namespace srxl2smart;

void setUp() {}
void tearDown() {}

void test_handshake_frame()
{
    uint8_t frame[14] = {};
    TEST_ASSERT_EQUAL_UINT32(14, buildHandshake(frame, 0x10, 0x40, 20, 0, 1, 0x12345678));
    TEST_ASSERT_EQUAL_HEX8(0xA6, frame[0]);
    TEST_ASSERT_EQUAL_HEX8(0x21, frame[1]);
    TEST_ASSERT_EQUAL_HEX8(14, frame[2]);
    TEST_ASSERT_EQUAL_HEX8(0x10, frame[3]);
    TEST_ASSERT_EQUAL_HEX8(0x40, frame[4]);
    TEST_ASSERT_TRUE(validateFrame(frame, sizeof(frame)));
}

void test_channel_frame_and_reserved_bits()
{
    uint8_t frame[16] = {};
    TEST_ASSERT_EQUAL_UINT32(16, buildChannel(frame, 0x40, 0xFFFF, false));
    TEST_ASSERT_EQUAL_HEX8(0xCD, frame[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame[3]);
    TEST_ASSERT_EQUAL_HEX8(0x40, frame[4]);
    TEST_ASSERT_EQUAL_HEX8(0xFC, frame[12]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame[13]);
    TEST_ASSERT_TRUE(validateFrame(frame, sizeof(frame)));
}

void test_decode_esc_telemetry()
{
    uint8_t payload[16] = {
        0x20, 0x00,
        0x04, 0xD2,
        0x08, 0xAC,
        0x01, 0xC2,
        0x04, 0xD2,
        0x01, 0x2C,
        0x0A, 0x64, 0x64, 0x64
    };
    SmartEscTelemetry telemetry;
    TEST_ASSERT_TRUE(decodeTelemetryPayload(payload, telemetry));
    TEST_ASSERT_EQUAL_UINT32(12340, telemetry.rpm);
    TEST_ASSERT_EQUAL_UINT16(222, telemetry.voltageDeciVolts);
    TEST_ASSERT_EQUAL_UINT16(123, telemetry.currentDeciAmps);
    TEST_ASSERT_EQUAL_INT16(450, telemetry.fetTemperatureDeciC);
    TEST_ASSERT_EQUAL_INT16(300, telemetry.becTemperatureDeciC);
}

void test_decode_capacity_telemetry()
{
    uint8_t payload[16] = {
        0x34, 0x00,
        0x00, 0x7B,
        0x04, 0xD2,
        0x01, 0xC2,
        0, 0, 0, 0, 0, 0, 0, 0
    };
    SmartEscTelemetry telemetry;
    TEST_ASSERT_TRUE(decodeTelemetryPayload(payload, telemetry));
    TEST_ASSERT_EQUAL_UINT16(123, telemetry.currentDeciAmps);
    TEST_ASSERT_EQUAL_UINT32(1234, telemetry.capacityMah);
    TEST_ASSERT_EQUAL_INT16(450, telemetry.fetTemperatureDeciC);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_handshake_frame);
    RUN_TEST(test_channel_frame_and_reserved_bits);
    RUN_TEST(test_decode_esc_telemetry);
    RUN_TEST(test_decode_capacity_telemetry);
    return UNITY_END();
}
