#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <cstring>
#include <cstdint>

#define ESP32_I2C_ADDRESS 0x08

class RCInputI2CDriver : public device::I2C , public I2CSPIDriver<RCInputI2CDriver>
{
public:
    RCInputI2CDriver(const I2CSPIDriverConfig &config);
    ~RCInputI2CDriver();

    int init() override;

    static void print_usage();

    void print_status() override;

    void RunImpl();

private:
    void sendRCDataToESP32();
    void Start();

    uORB::Subscription _rc_input_sub{ORB_ID(input_rc)};
    input_rc_s _rc_data{};
};

RCInputI2CDriver::RCInputI2CDriver(const I2CSPIDriverConfig &config):
    I2C(config),
    I2CSPIDriver(config)
{
}

RCInputI2CDriver::~RCInputI2CDriver()
{
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


void RCInputI2CDriver::Start()
{
    // Schedule a cycle to start things
    ScheduleNow();
}

void RCInputI2CDriver::RunImpl()
{
    sendRCDataToESP32();
    ScheduleDelayed(100000); // 10 Hz
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

void RCInputI2CDriver::print_status()
{
	I2CSPIDriverBase::print_status();
}

void RCInputI2CDriver::print_usage()
{
    PRINT_MODULE_USAGE_NAME("i2c32", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("esp32");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x08);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

// Main entry point for the module
extern "C" __EXPORT int i2c32_main(int argc, char *argv[])
{
    //int i2c_bus = 3; // Adjust the bus as needed

    using ThisDriver = RCInputI2CDriver;
    BusCLIArguments cli{true, false};
    cli.i2c_address = 0x08;
    cli.default_i2c_frequency = 400 * 1000;


    const char *verb = cli.parseDefaultArguments(argc, argv);

    if (!verb) {
	ThisDriver::print_usage();
	return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ESP32_DEVTYPE_I2C);


    if (!strcmp(verb, "start")) {
	return ThisDriver::module_start(cli, iterator);
    }

    if (!strcmp(verb, "stop")) {
	return ThisDriver::module_stop(iterator);
    }

    if (!strcmp(verb, "status")) {
	return ThisDriver::module_status(iterator);
    }

    ThisDriver::print_usage();
    return -1;
}
