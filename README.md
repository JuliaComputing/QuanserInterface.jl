# QuanserInterface

[![Build Status](https://github.com/baggepinnen/QuanserInterface.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/baggepinnen/QuanserInterface.jl/actions/workflows/CI.yml?query=branch%3Amain)

This repo contains a Julia interface to the Quanser hardware-in-the-loop (HIL) SDK. This allows you to control their devices, reading measurements and issuing control commands etc. 

## Installation

1. Install the hardware-in-the-loop (HIL) interface from here https://github.com/quanser/hil_sdk_linux_x86_64 (change linux to what's appropriate for your system)
2. To use the `PythonBackend` Install Quanser python packages as described [here](https://docs.quanser.com/quarc/documentation/python/installation.html) and manually install and load PythonCall (the python backend is an extension). Optionally, set the default backend using `QuanserInterface.set_default_backend("python")`.
3. To use the C backend (default), install the sdk, and if on Linux, possibly symlink `sudo ln -s /usr/lib/x86_64-linux-gnu/libquanser_communications.so.1 /lib/libquanser_communications.so` (or wherever the library is located on your system), I had issues with the `.1` suffix causing Libdl not to find the library. The easiest way to install all the required shared libraries is to follow the python install instructions, i.e., issue the `sudo apt install python3-quanser-apis` after having added their package server.

### Setting preferences
Preferences.jl is used to store the default backend choice, the path to the python HIL SDK as well as the default board type. You can set these preferences by running
```julia
QuanserInterface.set_default_backend(backend) # "c" or "python"
QuanserInterface.set_board(board)
QuanserInterface.set_quanser_python_path(path) # Only applies if python backend is installed
```

The defaults if none are set are equivalent to
```julia
QuanserInterface.set_default_backend("c")
QuanserInterface.set_board("qube_servo3_usb")
QuanserInterface.set_quanser_python_path("~/quanser")
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

## Run on hardware
If you have a Qube servo 3 connected to your machine, you can run one of the demos in the `examples` folder. The procedure is as follows
1. Instantiate a device object, e.g. `process = QubeServo()` or `process = QubeServoPendulum()`
2. Home the device (calibrate measurement offsets) by calling `home!`. This is only required for the `QubeServoPendulum()`, (unless you're interested in position control of the `QubeServo`). `home!` expects the arm of the device to be all the way to the left, hitting the end stop (if you are looking at the front logo of the device). The pendulum is expected to hang straight down. You can optionally provide the angle in degrees to `home!`, in this case, positive angles are to the left of zero (seen from above / right-hand rule with thumb pointing down). This must be done each time the device is powered on, and sometimes has to be redone when the device freaks out. There is also a function `go_home` which tries to move the arm to angle zero using a simple PI controller. 
3. Call `measure(process)` to read some measurements from the device.
4. Call `control(process, [0.5])` to issue a control command to the device. The `0.5` here should make the device move to positive angles (to the left). Reset by calling `control(process, [0.0])`.
5. Try one of the examples.

Method signatures in this package are typically picky with types, e.g., Ints are not accepted where `Float64` is required by the device.


## Implementing control loops
The macro `@periodically` is available to help building a loop with consistent and specified timing. See the examples for how to use it.

## Hardware safety
Some control loops in the examples directory contain a check for if the arm angle is out of bounds, and terminates the experiment by issuing `control(process, [0.0])` and returning if it is. This tries to prevent bad tuning from tearing the pendulum off the device or slamming into the end stops. Poorly tuned controllers can easily cause the magnetic connection to fail.

## Glossary
- `Ts` sample time, the time interval at which control loops are run. Typically in the range of 50-200Hz ($T_s \in [0.02, 0.005]$) for the Qube-Servo 3.
- `Tf` final time (duration).
- `y` measurement
- `u` control command
- `x` state
- `xh` state estimate ($\hat x$)
- `r` reference
- `e` error
- `K` proportional controller gain, sometimes also denotes a Kalman gain matrix
- `Ki` integral controller gain
- `Kf` Coulomb friction compensation parameter
- `L` state-feedback gain matrix $u = -Lx$ or $u = L(x_r-x)$
- `Q1, Q2` cost matrices for the LQR problem $J = \int_0^\infty x^T Q_1 x + u^T Q_2 u dt$
- `R1, R2` covariance matrices for the Kalman filter
