# M113 direct CANopen controller

`m113` replaces the external ESP32 I2C-to-CAN bridge. It reads `input_rc`
directly in PX4 and drives the actuator CAN bus at 1 Mbit/s.

The default `px4_fmu-v6x` startup uses CAN1. Connect the actuator CAN
transceiver to the first PX4 CAN interface. The UAVCAN daemon is not built for
this board: the M113 controller owns that CAN peripheral while it is running.

The runtime command is:

```sh
m113 start
m113 status
m113 stop
```

Use `m113 start -i 2` only when the actuator bus is wired to CAN2 and that
interface exists on the board variant.
