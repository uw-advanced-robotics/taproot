# Taproot Changelog

## November 2021

### Breaking changes

- The `sim-modm` directory in a generated Taproot instance has a new structure. Files are now in
  `sim-modm/hosted-TARGET/modm`, where `TARGET` is `linux`, `windows` or `darwin`. Make sure to
  delete and cleanly re-generate your Taproot instance, and update your SConstruct file as shown in
  the template project.
- The `ErrorController` now no longer displays errors on the LEDs of the RoboMaster Type A board.
  Now, to create an error using the `RAISE_ERROR` macro, you only pass in a pointer to a
  `tap::Drivers` object and a description (i.e. `RAISE_ERROR(drivers, "crc failure")`).

### All changes

- "sim-modm" instance is now generated for all three major desktop platforms, with hardware builds
  and testing environments fully supported on each. (!73, #96, #15)
- `CanRxHandler` class now supports can ids between `0x1e4` and `0x224`. (!84, #124)
