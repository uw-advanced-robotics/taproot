.. _controllers:

===========
Controllers
===========

This module documents the closed-loop control algorithms available in Taproot.

.. contents:: Table of Contents
   :local:
   :depth: 2

Smooth PID
----------

The Smooth PID controller is a variation of the standard PID algorithm that incorporates Kalman filters. It filters the proportional and derivative error terms to reduce noise amplification, making it suitable for systems with noisy sensors.

.. doxygenstruct:: tap::algorithms::SmoothPidConfig
   :project: taproot
   :members:

.. doxygenclass:: tap::algorithms::SmoothPid
   :project: taproot
   :members:

Fuzzy PD
--------

The Fuzzy PD controller inherits from the Smooth PID but adds fuzzy logic capabilities. It adaptively tunes the Proportional (P) and Derivative (D) gains at runtime based on the current error state, allowing the controller to handle non-linearities like static friction (stiction) or varying loads more effectively.

.. doxygenstruct:: tap::algorithms::FuzzyPDConfig
   :project: taproot
   :members:

.. doxygenclass:: tap::algorithms::FuzzyPD
   :project: taproot
   :members:

Lag-Lead Compensator
--------------------

The Lag-Lead implementation generates coefficients for a generic Discrete Filter. It is used to shape the frequency response of a system, adding poles and zeros to adjust phase margins and low-frequency gain.

.. doxygenfile:: lag_lead.hpp
   :project: taproot