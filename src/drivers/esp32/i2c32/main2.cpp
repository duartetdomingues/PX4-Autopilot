#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module_params.h>
#include <drivers/device/i2c.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <cstring>
#include <cstdint>

#define ESP32_I2C_ADDRESS 0x08

class RCInputI2CDriver : public device::I2C
{
public:
    RCInputI2CDriver(const int bus, const uint16_t address);
    virtual ~RCInputI2CDriver();

    int init();
    void Run();

private:
    void sendRCDataToESP32();
    void serializeRCInput(uint8_t *buffer, const input_rc_s &rc_data);

    uORB::Subscription _rc_input_sub{ORB_ID(input_rc)};
    input_rc_s _rc_data{};
};

RCInputI2CDriver::RCInputI2CDriver(const int bus, const uint16_t address)
    : I2C(DRV_ESP32_DEVTYPE_I2C, nullptr, bus, address, 100000)
{
}

RCInputI2CDriver::~RCInputI2CDriver()
{
    // Cleanup, if necessary
}

int RCInputI2CDriver::init()
{
    // Initialize the I2C device
    int ret = I2C::init();
    if (ret != PX4_OK) {
        PX4_ERR("I2C initialization failed");
        return ret;
    }

    PX4_INFO("RCInputI2CDriver initialized on bus %d, address 0x%02X", get_device_bus(), get_device_address());
    return PX4_OK;
}

void RCInputI2CDriver::serializeRCInput(uint8_t *buffer, const input_rc_s &rc_data)
{
    PX4_INFO("RCdata0: %d", rc_data.values[0]);
    PX4_INFO("RCdata1: %d", rc_data.values[1]);

    // Serialize timestamp
    buffer[0] = rc_data.timestamp & 0xFF;
    buffer[1] = (rc_data.timestamp >> 8) & 0xFF;
    buffer[2] = (rc_data.timestamp >> 16) & 0xFF;
    buffer[3] = (rc_data.timestamp >> 24) & 0xFF;

    // Serialize channel count
    buffer[4] = rc_data.channel_count;

    // Serialize channel values (max 18 channels)
    for (int i = 0; i < rc_data.channel_count && i < 18; ++i) {
        buffer[5 + 2 * i] = rc_data.values[i] & 0xFF;
        buffer[5 + 2 * i + 1] = (rc_data.values[i] >> 8) & 0xFF;
    }
}

void RCInputI2CDriver::sendRCDataToESP32()
{
    // Check if there is new data available in the rc_input topic
    if (_rc_input_sub.update(&_rc_data)) {
        uint8_t buffer[64] = {0}; // Prepare a buffer to serialize the data
        serializeRCInput(buffer, _rc_data);

        // Send data over I2C
        int ret = transfer(buffer, sizeof(buffer), nullptr, 0);
        if (ret != PX4_OK) {
            PX4_ERR("I2C transfer failed");
        } else {
            PX4_INFO("Data sent to ESP32 successfully");
        }
    }
}

void RCInputI2CDriver::Run()
{
    while (true) {
        sendRCDataToESP32();
        px4_usleep(100000); // Send data every 100 ms
    }
}

// Main entry point for the module
extern "C" __EXPORT int i2c32_main(int argc, char *argv[])
{
    int i2c_bus = 2; // Adjust the bus as needed
    RCInputI2CDriver driver(i2c_bus, ESP32_I2C_ADDRESS);

    if (driver.init() != PX4_OK) {
        PX4_ERR("Failed to initialize RCInputI2CDriver");
        return PX4_ERROR;
    }

    PX4_INFO("Starting RCInputI2CDriver loop");
    driver.Run();

    return PX4_OK;
}
