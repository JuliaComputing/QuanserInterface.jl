# QuanserInterface

[![Build Status](https://github.com/baggepinnen/QuanserInterface.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/baggepinnen/QuanserInterface.jl/actions/workflows/CI.yml?query=branch%3Amain)

This repo contains a Julia interface to the Quanser hardware-in-the-loop (HIL) SDK. This allows you to control their devices, reading measurements and issuing contorl commands etc. 

## Installation

1. Install the hardware-in-the-loop (HIL) interface from here https://github.com/quanser/hil_sdk_linux_x86_64 (change linux to what's appropriate for your system)
2. To use the `PythonBackend` Install Quanser python packages as described [here](https://docs.quanser.com/quarc/documentation/python/installation.html)
3. To use the C backend, implement the C backend

## Supported devices
The currently supported devices are
- Qube-Servo 3 with pendulum or inertia disc attachment

Adding additional devices should be straightforward.

## Interface
The interface follows that of [HardwareAbstractions.jl](https://github.com/baggepinnen/HardwareAbstractions.jl), defining methods for
- `HardwareAbstractions.measure`
- `HardwareAbstractions.control`

For both physical and simulated version of the devices. 
