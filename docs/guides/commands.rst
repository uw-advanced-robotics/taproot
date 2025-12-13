Command Composition & Bindings
==============================

Taproot provides several specialized classes to combine commands or bind them to specific user inputs (Remote, Keyboard, Mouse).

Command Composition
-------------------
These classes allow you to group multiple commands together to create complex behaviors, such as running actions in parallel or creating sequential state machines.

.. doxygenclass:: tap::control::ConcurrentTemplateCommand
   :project: taproot
   :members:

.. doxygenclass:: tap::control::ComprisedCommand
   :project: taproot
   :members:

Command Mappings
----------------
Mappings determine how a physical input triggers a command. These are used in conjunction with the ``CommandMapper``.

.. doxygenclass:: tap::control::PressCommandMapping
   :project: taproot
   :members:

.. doxygenclass:: tap::control::HoldCommandMapping
   :project: taproot
   :members:

.. doxygenclass:: tap::control::HoldRepeatCommandMapping
   :project: taproot
   :members:

.. doxygenclass:: tap::control::ToggleCommandMapping
   :project: taproot
   :members:

Input Configuration
-------------------
Helper classes used to define the specific inputs (Keys, Switches, Mouse Buttons) required to trigger a mapping.

.. doxygenclass:: tap::control::RemoteMapState
   :project: taproot
   :members: