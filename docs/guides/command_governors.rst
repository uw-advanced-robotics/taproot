Command Governors
=================

Governors provide a mechanism to gate command execution based on external logic or system state (e.g., heat levels, sensor data, or safety interlocks). Instead of embedding this logic directly into a Command, you can encapsulate it in a Governor and wrap the Command.

Governor Interface
------------------
The base interface for creating logic gates. Override ``isReady()`` to determine when a command can start, and ``isFinished()`` to determine if a running command must be forcibly stopped.

.. doxygenclass:: tap::control::governor::CommandGovernorInterface
   :project: taproot
   :members:

Governor Limited Command
------------------------
A wrapper that only executes the contained command if **all** provided governors are ready. It will also finish the command early if **any** governor signals it is finished.

.. doxygenclass:: tap::control::governor::GovernorLimitedCommand
   :project: taproot
   :members:

Governor With Fallback Command
------------------------------
A wrapper that toggles between two commands based on the governors. If the governors are ready, it runs the "Primary" command. If not, it runs the "Fallback" command.

.. doxygenclass:: tap::control::governor::GovernorWithFallbackCommand
   :project: taproot
   :members: