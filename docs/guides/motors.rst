Motor Subsystem
===============

The motor subsystem provides drivers for interacting with brushless DC motors, specifically optimized for the DJI RoboMaster series (M3508, M2006, GM6020) which communicate over CAN bus.

Motor Interface
---------------
The base interface that all motor drivers must implement. This abstraction allows control algorithms to be written without dependency on specific hardware implementations.

.. doxygenclass:: tap::motor::MotorInterface
   :project: taproot
   :members:

DJI Motor Drivers
-----------------
These classes implement the `MotorInterface` for DJI intelligent ESCs and motors.

**DJI Motor**
The standard driver for a single DJI motor (e.g., C620/M3508, C610/M2006, GM6020). It handles parsing incoming CAN feedback (position, velocity, torque, temp) and preparing control commands.

.. doxygenclass:: tap::motor::DjiMotor
   :project: taproot
   :members:

**Double DJI Motor**
A wrapper class for two identical motors that are mechanically rigidly linked (driving the same shaft). It presents a single `MotorInterface` to the user but duplicates output commands to both physical motors.

.. doxygenclass:: tap::motor::DoubleDjiMotor
   :project: taproot
   :members:

Encoders & Communication
------------------------
Low-level helper classes used by the motor drivers to handle CAN protocol details and sensor wrapping.

**DJI Motor Encoder**
Handles the interpretation of the raw encoder data sent by DJI motors over CAN. It manages wrapping (handling the 0-8191 overflow) and gear ratio calculations.

.. doxygenclass:: tap::motor::DjiMotorEncoder
   :project: taproot
   :members:

**DJI Motor TX Handler**
Manages the transmission of motor commands. The DJI CAN protocol packs commands for up to 4 motors into a single CAN frame. This class aggregates commands from multiple `DjiMotor` instances and sends them efficiently.

.. doxygenclass:: tap::motor::DjiMotorTxHandler
   :project: taproot
   :members: