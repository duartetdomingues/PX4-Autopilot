#include <drivers/device/i2c.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>


extern "C"{ __EXPORT int i2c32_main(int argc, char *argv[]);}

class MotorDriver : public device::I2C, public I2CSPIDriver<MotorDriver>, public px4::WorkItem
{
public:
    //MotorDriver(int bus = 0, uint8_t address = 0x08);  // Default I2C address: 0x08
    const I2CSPIDriverConfig &config;
    MotorDriver();
    ~MotorDriver();

    int init() override;

private:
    int write_motor_command(uint8_t motor_id, uint16_t command);
    void handle_rc_input();

    uint8_t _i2c_address;
    uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
};

/* MotorDriver::MotorDriver(int bus, uint8_t address)
    : I2C(DRV_ESP32_DEVTYPE_I2C, "/dev/motordriver", bus, address, 100000),
      WorkItem(MODULE_NAME, px4::device_bus_to_wq(device::I2C::get_device_id())),
      _i2c_address(address)
{} */

MotorDriver::MotorDriver(const I2CSPIDriverConfig &config)
   	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	px4::WorkItem(DRV_ESP32_DEVTYPE_I2C, px4::device_bus_to_wq(device::I2C::get_device_id()))
{}

MotorDriver::~MotorDriver()
{

}

int MotorDriver::init()
{
    if (I2C::init() != PX4_OK) {
        PX4_ERR("Failed to initialize I2C with address: 0x%02X", _i2c_address);
        return PX4_ERROR;
    }
    PX4_INFO("I2C initialized with address: 0x%02X", _i2c_address);
    return PX4_OK;
}

void MotorDriver::Run()
{
    // Handle RC input and send motor commands
    handle_rc_input();
}

void MotorDriver::handle_rc_input()
{
    input_rc_s input_rc;
    if (_input_rc_sub.copy(&input_rc)) {

        // Send commands to motors
	for (int i = 0; i < 18; i++) {
	    write_motor_command(i, input_rc.values[i]);
	}

    }
}

int MotorDriver::write_motor_command(uint8_t motor_id, uint16_t command)
{
    uint8_t data[3] = {
        motor_id,               // First byte: Motor ID
        static_cast<uint8_t>(command >> 8), // Second byte: High byte of uint16_t
        static_cast<uint8_t>(command & 0xFF) // Third byte: Low byte of uint16_t
    };

    return transfer(data, sizeof(data), nullptr, 0);
}


int i2c32_main(int argc, char *argv[])
{
	PX4_INFO("i2c32_main started");

	MotorDriver motor_driver;
	if (motor_driver.init() != PX4_OK) {
		PX4_ERR("MotorDriver initialization failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}
