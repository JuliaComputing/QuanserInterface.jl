# QuanserInterface

[![Build Status](https://github.com/baggepinnen/QuanserInterface.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/baggepinnen/QuanserInterface.jl/actions/workflows/CI.yml?query=branch%3Amain)

This repo contains a Julia interface to the Quanser hardware-in-the-loop (HIL) SDK. This allows you to control their devices, reading measurements and issuing contorl commands etc. 

## Installation

1. Install the hardware-in-the-loop (HIL) interface from here https://github.com/quanser/hil_sdk_linux_x86_64 (change linux to what's appropriate for your system)
2. To use the `PythonBackend` Install Quanser python packages as described [here](https://docs.quanser.com/quarc/documentation/python/installation.html)
3. To use the C backend, implement the C backend

### Setting preferences
Preferences.jl is used to store the path to the python HIL SDK as well as the default board type. You can set these preferences by running
```julia
QuanserInterface.set_quanser_python_path(path)
QuanserInterface.set_board(board)
```

The defaults if none are set are equivalent to
```julia
QuanserInterface.set_quanser_python_path("~/quanser")
QuanserInterface.set_board("qube_servo3_usb")
```

## Supported devices
The currently supported devices are
- Qube-Servo 3 with pendulum or inertia disc attachment

Adding additional devices should be straightforward.

## Interface
The interface follows that of [HardwareAbstractions.jl](https://github.com/baggepinnen/HardwareAbstractions.jl), defining methods for
- `HardwareAbstractions.measure`
- `HardwareAbstractions.control`

For both physical and simulated version of the devices. 
