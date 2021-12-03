# Taproot Changelog

## December 2021

### Breaking changes

- Almost everything in `tap::control::setpoint` has changed. Most will faily loudly (i.e.: will cause
  compilation errors). Those that are potentially more insidious are documented below:
  - Order of parameters in constructor for `tap::control::setpoint::MoveUnjamComprisedCommand` changed
    (grouped by usage now) 

## November 2021

### Breaking changes

- The `sim-modm` directory in a generated Taproot instance has a new structure. Files are now in
  `sim-modm/hosted-TARGET/modm`, where `TARGET` is `linux`, `windows` or `darwin`. Make sure to
  delete and cleanly re-generate your Taproot instance, and update your SConstruct file as shown in
  the template project.
- The `ErrorController` now no longer displays errors on the LEDs of the RoboMaster Type A board.
  Now, to create an error using the `RAISE_ERROR` macro, you only pass in a pointer to a
  `tap::Drivers` object and a description (i.e. `RAISE_ERROR(drivers, "crc failure")`).
- Minor cleanup to the `RefSerial` object. Some fields in the `RefSerial` class now use
  `MODM_FLAGSX`. See [modm's
  documentation](https://modm.io/reference/module/modm-architecture-register/) for how these work.
  Structs and enum values previously stored directly inside the `RefSerial` object now must be
  prefixed by `Rx` or `Tx`.
- `tap::communication::serial::ITerminalSerialCallback` interface renamed to
  `tap::communication::TerminalSerialCallbackInterface`.

### All changes

- "sim-modm" instance is now generated for all three major desktop platforms, with hardware builds
  and testing environments fully supported on each. (!73, #96, #15)
- Tests were added to the referee serial class. Minor logic/bug changes were made based on the
  tests (!80, #80).
- Robot-to-robot interaction handling was added to the `RefSerial` class. One can send a
  robot-to-robot message via `sendRobotToRobotMsg` and register a callback with the `RefSerial`
  object via `attachRobotToRobotMessageHandler`. This functionality is still in the beta-testing
  phase and needs further validation (!80, #80).
- `CanRxHandler` class now supports can ids between `0x1e4` and `0x224`. (!84, #124)
- Tests added to terminal serial and various bugs in related classes were removed (!67, #58).
