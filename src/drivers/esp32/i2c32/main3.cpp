#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/i2c.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <cstring>
#include <cstdint>

#define ESP32_I2C_ADDRESS 0x08

class RCInputI2CDriver : public device::I2C , public I2CSPIDriver<RCInputI2CDriver>
{
public:
    RCInputI2CDriver(const int bus, const uint16_t address);
    virtual ~RCInputI2CDriver();

    int init(); override;
    void RunImpl();
    void Start();

private:
    void sendRCDataToESP32();

    uORB::Subscription _rc_input_sub{ORB_ID(input_rc)};
    input_rc_s _rc_data{};
};

RCInputI2CDriver::RCInputI2CDriver(const int bus, const uint16_t address)
    : I2C(DRV_ESP32_DEVTYPE_I2C, "ESP32_I2C", bus, address, 400000)
	    , I2CSPIDriver(bus, address)
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

    // Start the driver
    Start();

    PX4_INFO("RCInputI2CDriver initialized on bus %d, address 0x%02X", get_device_bus(), get_device_address());
    return PX4_OK;
}

void RCInputI2CDriver::sendRCDataToESP32()
{
    input_rc_s input_rc;
    if (_rc_input_sub.copy(&input_rc)) {

        //18*uint16_t
        uint8_t send_data[36];

        // Send commands to motors
        for (int i = 0; i < 36;i++) {
            send_data[i] = static_cast<uint8_t>(input_rc.values[i/2] >> 8);
		    i++;
            send_data[i] = static_cast<uint8_t>(input_rc.values[i/2] & 0xFF);
        }

        int req = transfer(send_data, sizeof(send_data), nullptr, 0);
	    if (req != PX4_OK)
		    PX4_ERR("Failed to send motor command");
	    else
		    PX4_INFO("Sent motor command");

    }
}

void RCInputI2CDriver::RunImpl()
{
    sendRCDataToESP32();
    ScheduleDelayed(100000); // 10 Hz
}
void RCInputI2CDriver::Run()
{
    RunImpl();
    PX4_INFO("Task executed!");
}

void RCInputI2CDriver::Start()
{
    // Schedule a cycle to start things
	Run();
    ScheduleNow();
}

// Main entry point for the module
extern "C" __EXPORT int i2c32_main(int argc, char *argv[])
{
    int i2c_bus = 3; // Adjust the bus as needed
    RCInputI2CDriver  ThisDriver(i2c_bus, ESP32_I2C_ADDRESS);

    if ( ThisDriver.init() != PX4_OK) {
        PX4_ERR("Failed to initialize RCInputI2CDriver");
        return PX4_ERROR;
    }

    PX4_INFO("Starting RCInputI2CDriver loop");
    ThisDriver.Start();

    return PX4_OK;
}
