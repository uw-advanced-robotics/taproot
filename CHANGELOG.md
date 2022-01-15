# Taproot Changelog

## January 2022

### Breaking changes
- `tap::controls::ControlOperatorInterface` has been removed from Taproot. We have added it to
  [our personal open-source project, aruw-mcb](https://gitlab.com/aruw/controls/aruw-mcb) for those
  who would like to keep up with our implementation of that feature, but it will need to be
  added/implemented externally going forward in order to keep the functionality it provides.
- `tap::controls::chassis::PowerLimiter` API changed significantly. The constructor now takes in
  less parameters than before and their purpose is different. Also, rather than modifying motor
  outputs directly, the power limiter returns a fraction and it is the user's responsibility to
  multiply the motor output by this fraction.

### All changes

- `DjiMotor` comments improved (!95).
- Small improvement to command scheduler subsystem refresh loop logic.
- `tap::communication::sensors::current::CurrentSensorInterface` and `AnalogCurrentSensor` added,
  which are software constructs for current sensors.
- Power limiting logic improved and simplified, interfaces with a generic `CurrentSensorInterface`
  (!92). 
- `tap::controls::ControlOperatorInterface` deprecated.

## December 2021

### Breaking changes

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
- The drivers object is now generated in `taproot/src/tap` rather than in some user directory. To
  append your own drivers to the `tap::Drivers` object, inherit `tap::Drivers`.

### All changes

- Tests were added to the referee serial class. Minor logic/bug changes were made based on the tests
  (!80, #80).
- Robot-to-robot interaction handling was added to the `RefSerial` class. One can send a
  robot-to-robot message via `sendRobotToRobotMsg` and register a callback with the `RefSerial`
  object via `attachRobotToRobotMessageHandler`. This functionality is still in the beta-testing
  phase and needs further validation (!80, #80).
- `CanRxHandler` class now supports can ids between `0x1e4` and `0x224`. (!84, #124)
- Tests added to terminal serial and various bugs in related classes were removed (!67, #58).
- The `CommandScheduler` is now able to safely remove all commands when a user-specified
  "disconnected" state occurs. One can pass a `SafeDisconnectFunction` functor to the
  `CommandScheduler` to determine what causes a "disconnected" state (!75).
- `modm:math:interpolation` module now generated (!93).

## November 2021

### Breaking changes

- The `sim-modm` directory in a generated Taproot instance has a new structure. Files are now in
  `sim-modm/hosted-TARGET/modm`, where `TARGET` is `linux`, `windows` or `darwin`. Make sure to
  delete and cleanly re-generate your Taproot instance, and update your SConstruct file as shown in
  the template project.

### All changes

- "sim-modm" instance is now generated for all three major desktop platforms, with hardware builds
  and testing environments fully supported on each. (!73, #96, #15)
