.. _transforms:

==========
Transforms
==========

The Transforms module provides a comprehensive system for handling 3D spatial mathematics, coordinate frame transformations, and kinematics. It is built upon the CMSIS DSP library for efficient matrix operations.

This system distinguishes between **Static Transforms** (pure translation and rotation) and **Dynamic Transforms** (which include velocity and acceleration).

.. contents:: Table of Contents
   :local:
   :depth: 2

The Transform Class
-------------------

The ``Transform`` class is the core component of this module. It represents the relationship between two coordinate frames (a "follower" frame relative to a "base" frame). It handles both the geometric transformation of points and vectors, as well as the kinematic projection of frames over time.

.. doxygenclass:: tap::algorithms::transforms::Transform
   :project: taproot
   :members:

Geometric Primitives
--------------------

These classes provide the fundamental 3D mathematical structures used by the Transform system.

Vector
^^^^^^

A wrapper around a 3x1 column matrix representing a vector in 3D space.

.. doxygenclass:: tap::algorithms::transforms::Vector
   :project: taproot
   :members:

Orientation
^^^^^^^^^^^

Represents a 3D rotation, internally stored as a 3x3 rotation matrix. It supports construction from Euler angles (Roll, Pitch, Yaw).

.. doxygenclass:: tap::algorithms::transforms::Orientation
   :project: taproot
   :members:

Angular Velocity
^^^^^^^^^^^^^^^^

Represents the rate of change of orientation, stored as a skew-symmetric matrix to facilitate efficient physics calculations.

.. doxygenclass:: tap::algorithms::transforms::AngularVelocity
   :project: taproot
   :members:

Dynamic States
--------------

These container classes bundle position and orientation with their respective time derivatives (velocity and acceleration). They are used to initialize or update Dynamic Transforms.

Dynamic Position
^^^^^^^^^^^^^^^^

Combines translation (Position), linear velocity (Vector), and linear acceleration (Vector).

.. doxygenclass:: tap::algorithms::transforms::DynamicPosition
   :project: taproot
   :members:

Dynamic Orientation
^^^^^^^^^^^^^^^^^^^

Combines attitude (Orientation) and rotational rate (AngularVelocity).

.. doxygenclass:: tap::algorithms::transforms::DynamicOrientation
   :project: taproot
   :members: