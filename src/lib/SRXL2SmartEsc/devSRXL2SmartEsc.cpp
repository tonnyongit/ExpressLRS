#include "targets.h"

#if defined(TARGET_RX) && defined(PLATFORM_ESP32)

#include "devSRXL2SmartEsc.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <soc/soc_caps.h>
#include <soc/uart_pins.h>

#include "CRSFRouter.h"
#include "SRXL2Protocol.h"
#include "common.h"
#include "config.h"
#include "crsf_protocol.h"
#include "logging.h"
#include "rxtx_intf.h"

#if SOC_UART_NUM > 2

namespace
{
constexpr uint32_t SRXL2_BAUD = 115200;
constexpr uint8_t SRXL2_PRIORITY = 20;
constexpr uint8_t SRXL2_BAUD_SUPPORTED = 0x00;
constexpr uint8_t SRXL2_DEVICE_INFO = 0x01;
constexpr uint32_t STARTUP_DELAY_MS = 50;
constexpr uint32_t HANDSHAKE_STEP_MS = 8;
constexpr uint32_t HANDSHAKE_RETRY_MS = 300;
constexpr uint32_t CHANNEL_PERIOD_MS = 11;
constexpr uint32_t BUS_TIMEOUT_MS = 2000;
constexpr uint32_t BATTERY_PERIOD_MS = 200;
constexpr uint32_t RPM_PERIOD_MS = 100;
constexpr uint32_t TEMPERATURE_PERIOD_MS = 500;
constexpr uint32_t TELEMETRY_STALE_MS = 2500;
constexpr uint8_t UART_NUMBER = 2;

HardwareSerial srxl2Serial(UART_NUMBER);
int8_t srxl2Output = -1;
int8_t srxl2Pin = UNDEF_PIN;
bool uartStarted = false;

enum class BusState : uint8_t
{
    Startup,
    Handshake,
    Running,
};

BusState busState = BusState::Startup;
uint32_t stateStartedMs = 0;
uint32_t lastTxStartedMs = 0;
uint32_t lastValidRxMs = 0;
uint8_t scanDeviceId = srxl2smart::ESC_DEVICE_ID_MIN;
uint8_t escDeviceId = 0;
uint8_t receiveFrame[srxl2smart::MAX_PACKET_SIZE] = {};
size_t receiveLength = 0;
size_t expectedLength = 0;

srxl2smart::SmartEscTelemetry telemetry;
uint32_t lastTelemetryUpdateMs = 0;
uint32_t lastBatterySentMs = 0;
uint32_t lastRpmSentMs = 0;
uint32_t lastTemperatureSentMs = 0;

void setReceiveMode()
{
    pinMode(srxl2Pin, INPUT_PULLUP);
    pinMatrixInAttach(srxl2Pin, U2RXD_IN_IDX, false);
}

void setTransmitMode()
{
    pinMode(srxl2Pin, OUTPUT);
    digitalWrite(srxl2Pin, HIGH);
    pinMatrixOutAttach(srxl2Pin, U2TXD_OUT_IDX, false, false);
}

void resetParser()
{
    receiveLength = 0;
    expectedLength = 0;
}

void resetBus(uint32_t now)
{
    busState = BusState::Startup;
    stateStartedMs = now;
    lastTxStartedMs = 0;
    lastValidRxMs = 0;
    scanDeviceId = srxl2smart::ESC_DEVICE_ID_MIN;
    escDeviceId = 0;
    resetParser();
}

uint32_t receiverUid()
{
    const uint8_t *uid = config.GetUID();
    return static_cast<uint32_t>(uid[2]) |
           (static_cast<uint32_t>(uid[3]) << 8) |
           (static_cast<uint32_t>(uid[4]) << 16) |
           (static_cast<uint32_t>(uid[5]) << 24);
}

void sendFrame(const uint8_t *frame, size_t length, uint32_t now)
{
    if (length == 0 || length > srxl2smart::MAX_PACKET_SIZE)
    {
        return;
    }

    setTransmitMode();
    srxl2Serial.write(frame, length);
    srxl2Serial.flush();
    while (srxl2Serial.available() > 0)
    {
        srxl2Serial.read();
    }
    setReceiveMode();
    lastTxStartedMs = now;
}

void sendHandshake(uint8_t destination, uint32_t now)
{
    uint8_t frame[14];
    const size_t length = srxl2smart::buildHandshake(
        frame, srxl2smart::MASTER_DEVICE_ID, destination, SRXL2_PRIORITY,
        SRXL2_BAUD_SUPPORTED, SRXL2_DEVICE_INFO, receiverUid());
    sendFrame(frame, length, now);
}

uint16_t channelToSrxl2(uint32_t crsfValue)
{
    if (crsfValue == CRSF_CHANNEL_VALUE_UNSET)
    {
        return 0;
    }

    const uint16_t us = constrain<uint16_t>(
        CRSF_to_US(static_cast<uint16_t>(crsfValue)),
        static_cast<uint16_t>(US_CHANNEL_VALUE_STD_MIN),
        static_cast<uint16_t>(US_CHANNEL_VALUE_STD_MAX));
    uint32_t value;
    if (us <= US_CHANNEL_VALUE_CENTER)
    {
        value = static_cast<uint32_t>(us - US_CHANNEL_VALUE_STD_MIN) * 32768U /
                (US_CHANNEL_VALUE_CENTER - US_CHANNEL_VALUE_STD_MIN);
    }
    else
    {
        value = 32768U +
                static_cast<uint32_t>(us - US_CHANNEL_VALUE_CENTER) * (65532U - 32768U) /
                (US_CHANNEL_VALUE_STD_MAX - US_CHANNEL_VALUE_CENTER);
    }
    return static_cast<uint16_t>(value) & 0xFFFCU;
}

bool outputIsFailsafe()
{
    if (connectionState != connected || !connectionHasModelMatch ||
        !teamraceHasModelMatch || getLq() == 0)
    {
        return true;
    }

    const uint8_t inputChannel = config.GetPwmChannel(srxl2Output)->val.inputChannel;
    return ChannelData[inputChannel] == CRSF_CHANNEL_VALUE_UNSET;
}

void sendChannels(uint32_t now)
{
    const bool failsafe = outputIsFailsafe();
    uint16_t throttle = 0;
    if (!failsafe)
    {
        const uint8_t inputChannel = config.GetPwmChannel(srxl2Output)->val.inputChannel;
        throttle = channelToSrxl2(ChannelData[inputChannel]);
    }

    uint8_t frame[16];
    const size_t length = srxl2smart::buildChannel(
        frame, escDeviceId, throttle, failsafe, 0, 0);
    sendFrame(frame, length, now);
}

void publishBattery(uint32_t now)
{
    if ((!telemetry.voltageValid && !telemetry.currentValid && !telemetry.capacityValid) ||
        now - lastBatterySentMs < BATTERY_PERIOD_MS)
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_battery_t) battery{};
    battery.p.voltage = htobe16(telemetry.voltageValid ? telemetry.voltageDeciVolts : 0);
    battery.p.current = htobe16(telemetry.currentValid ? telemetry.currentDeciAmps : 0);
    battery.p.capacity = htobe24(telemetry.capacityValid ? telemetry.capacityMah : 0);
    battery.p.remaining = 0;
    crsfRouter.SetHeaderAndCrc(&battery.h, CRSF_FRAMETYPE_BATTERY_SENSOR,
                               CRSF_FRAME_SIZE(sizeof(crsf_sensor_battery_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &battery.h);
    crsfBatterySensorDetected = true;
    lastBatterySentMs = now;
}

void publishRpm(uint32_t now)
{
    if (!telemetry.rpmValid || now - lastRpmSentMs < RPM_PERIOD_MS)
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_rpm_t) rpm{};
    rpm.p.source_id = 0;
    rpm.p.rpm0 = htobe24(telemetry.rpm);
    constexpr uint8_t payloadLength = 4;
    crsfRouter.SetHeaderAndCrc(&rpm.h, CRSF_FRAMETYPE_RPM,
                               CRSF_FRAME_SIZE(payloadLength));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &rpm.h);
    lastRpmSentMs = now;
}

void publishTemperature(uint32_t now)
{
    if ((!telemetry.fetTemperatureValid && !telemetry.becTemperatureValid) ||
        now - lastTemperatureSentMs < TEMPERATURE_PERIOD_MS)
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_temp_t) temperature{};
    temperature.p.source_id = 0;
    uint8_t count = 0;
    if (telemetry.fetTemperatureValid)
    {
        temperature.p.temperature[count++] = htobe16(telemetry.fetTemperatureDeciC);
    }
    if (telemetry.becTemperatureValid)
    {
        temperature.p.temperature[count++] = htobe16(telemetry.becTemperatureDeciC);
    }

    const uint8_t payloadLength = static_cast<uint8_t>(1 + count * sizeof(uint16_t));
    crsfRouter.SetHeaderAndCrc(&temperature.h, CRSF_FRAMETYPE_TEMP,
                               CRSF_FRAME_SIZE(payloadLength));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &temperature.h);
    lastTemperatureSentMs = now;
}

void publishTelemetry(uint32_t now)
{
    if (lastTelemetryUpdateMs == 0 || now - lastTelemetryUpdateMs > TELEMETRY_STALE_MS)
    {
        return;
    }
    publishBattery(now);
    publishRpm(now);
    publishTemperature(now);
}

void handleFrame(const uint8_t *frame, size_t length, uint32_t now)
{
    if (!srxl2smart::validateFrame(frame, length))
    {
        return;
    }

    lastValidRxMs = now;

    if (frame[1] == srxl2smart::PACKET_HANDSHAKE && length >= 14)
    {
        const uint8_t source = frame[3];
        const uint8_t destination = frame[4];
        if (source >= srxl2smart::ESC_DEVICE_ID_MIN &&
            source <= srxl2smart::ESC_DEVICE_ID_MAX &&
            (destination == srxl2smart::MASTER_DEVICE_ID || destination == 0))
        {
            escDeviceId = source;
            DBGLN("SRXL2 SMART ESC discovered at device ID 0x%02X", escDeviceId);
        }
        return;
    }

    if (frame[1] == srxl2smart::PACKET_TELEMETRY && length >= 22)
    {
        if (frame[3] == 0xFF)
        {
            resetBus(now);
            return;
        }

        if (srxl2smart::decodeTelemetryPayload(&frame[4], telemetry))
        {
            lastTelemetryUpdateMs = now;
        }
    }
}

void consumeByte(uint8_t value, uint32_t now)
{
    if (receiveLength == 0)
    {
        if (value != srxl2smart::MAGIC)
        {
            return;
        }
        receiveFrame[receiveLength++] = value;
        return;
    }

    if (receiveLength >= sizeof(receiveFrame))
    {
        resetParser();
        return;
    }

    receiveFrame[receiveLength++] = value;
    if (receiveLength == 3)
    {
        expectedLength = receiveFrame[2];
        if (expectedLength < 5 || expectedLength > sizeof(receiveFrame))
        {
            resetParser();
        }
        return;
    }

    if (expectedLength != 0 && receiveLength == expectedLength)
    {
        handleFrame(receiveFrame, receiveLength, now);
        resetParser();
    }
}

void readSerial(uint32_t now)
{
    while (srxl2Serial.available() > 0)
    {
        consumeByte(static_cast<uint8_t>(srxl2Serial.read()), now);
    }
}

bool initialize()
{
    if (!OPT_HAS_SERVO_OUTPUT)
    {
        return false;
    }

    for (uint8_t output = 0; output < GPIO_PIN_PWM_OUTPUTS_COUNT; ++output)
    {
        if (config.GetPwmChannel(output)->val.mode == somSRXL2)
        {
            if (srxl2Output != -1)
            {
                DBGLN("Only one SRXL2 SMART ESC output is supported");
                return false;
            }
            srxl2Output = output;
            srxl2Pin = GPIO_PIN_PWM_OUTPUTS[output];
        }
    }

    if (srxl2Output == -1 || srxl2Pin == UNDEF_PIN)
    {
        return false;
    }

    srxl2Serial.setRxBufferSize(256);
    srxl2Serial.begin(SRXL2_BAUD, SERIAL_8N1, srxl2Pin, srxl2Pin, false);
    setReceiveMode();
    uartStarted = true;
    resetBus(millis());
    DBGLN("SRXL2 SMART ESC enabled on output %u (GPIO %d)", srxl2Output + 1, srxl2Pin);
    return true;
}

int start()
{
    return uartStarted ? 1 : DURATION_NEVER;
}

int event()
{
    if (connectionState == wifiUpdate && uartStarted)
    {
        srxl2Serial.end();
        pinMode(srxl2Pin, INPUT);
        uartStarted = false;
        return DURATION_NEVER;
    }
    return uartStarted ? 1 : DURATION_NEVER;
}

int timeout()
{
    if (!uartStarted)
    {
        return DURATION_NEVER;
    }

    const uint32_t now = millis();
    readSerial(now);
    publishTelemetry(now);

    switch (busState)
    {
    case BusState::Startup:
        if (now - stateStartedMs >= STARTUP_DELAY_MS)
        {
            busState = BusState::Handshake;
            stateStartedMs = now;
            scanDeviceId = srxl2smart::ESC_DEVICE_ID_MIN;
            sendHandshake(scanDeviceId, now);
        }
        break;

    case BusState::Handshake:
        if (now - lastTxStartedMs >= HANDSHAKE_STEP_MS)
        {
            if (scanDeviceId < srxl2smart::ESC_DEVICE_ID_MAX)
            {
                sendHandshake(++scanDeviceId, now);
            }
            else
            {
                sendHandshake(0xFF, now);
                busState = BusState::Running;
                stateStartedMs = now;
            }
        }
        break;

    case BusState::Running:
        if (escDeviceId == 0 && now - stateStartedMs >= HANDSHAKE_RETRY_MS)
        {
            resetBus(now);
            break;
        }
        if (!outputIsFailsafe() && lastValidRxMs != 0 && now - lastValidRxMs >= BUS_TIMEOUT_MS)
        {
            resetBus(now);
            break;
        }
        if (lastTxStartedMs == 0 || now - lastTxStartedMs >= CHANNEL_PERIOD_MS)
        {
            sendChannels(now);
        }
        break;
    }

    return 1;
}
} // namespace

#else

namespace
{
bool initialize() { return false; }
int start() { return DURATION_NEVER; }
int event() { return DURATION_NEVER; }
int timeout() { return DURATION_NEVER; }
} // namespace

#endif

device_t SRXL2SmartEsc_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONNECTION_CHANGED
};

#else

device_t SRXL2SmartEsc_device = {
    .initialize = nullptr,
    .start = nullptr,
    .event = nullptr,
    .timeout = nullptr,
    .subscribe = EVENT_NONE
};

#endif
