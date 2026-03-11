#! /usr/bin/env julia
#=
This script performs swingup of the pendulum using an energy-based controller, and stabilizes the pendulum at the top using an LQR controller. The controller gain is designed using furuta_lqg.jl
=#

if splitdir(Base.active_project())[1] != dirname(@__DIR__)
    @warn "Not in the QuanserInterface.jl/examples project, activating it"
    using Pkg; Pkg.activate(dirname(@__DIR__))
end

using QuanserInterface
using HardwareAbstractions
using ControlSystemsBase
using QuanserInterface: energy, measure
using StaticArrays
using WGLMakie


# Define some useful parameters and constants 
# to control the Quanser pendulum
const rr = Ref([0, pi, 0, 0])
nu  = 1     # number of controls
nx  = 4     # number of states
Ts  = 0.005 # sampling time

# Initialize the Quanser pendulum, your computer
# must be connected to it at this point.
process = QuanserInterface.QubeServoPendulum(; Ts)
rr[][1] = deg2rad(0)
rr[][2] = pi
# Run a test measurement, this returns two angles
# in radians, the first is the angle of the "block"/rotator,
# the second is the angle of the arm.
y = QuanserInterface.measure(process)

# Create a figure to visualize the pendulum.
fig = Figure()
# The `LScene` is a 3D graphics context on which we will plot the pendulum.
# This is currently quite primitive, but we can easily beautify it by using a `mesh!` plot
visualization_scene = LScene(fig[1, 1])
# The "block" points straight ahead
block_plot = lines!(visualization_scene, [Point3f(0, 0, 0), Point3f(1, 0, 0)]; linewidth = 5, color = :gray)
# The "arm" is assumed to be hanging down initially.
arm_plot = lines!(visualization_scene, [Point3f(1, 0, 0), Point3f(1, 0, -1)]; linewidth = 5, color = :red)

# This slider grid will allow us to control the force applied to the pendulum.
# You can add more named tuples to create more sliders, that can update more parameters.
sg = SliderGrid(
    fig[2, 1],
    (; label = "Force", range = LinRange(60, 90, 100), startvalue = 80.0, format = "{:.2f}");
    tellwidth = false,
    tellheight = true
)
# Slider observables are of type Any by default
force_untyped = sg.sliders[1].value
# but you can force it to be of a certain type
force = lift(Float64, force_untyped)

# This callback is called every time the figure is rendered.
# It updates the plots to reflect the current state of the pendulum.
on(events(fig).tick) do tick
    (block_angle, arm_angle) = QuanserInterface.measure(process)
    # Rotate the block around the z-axis
    block_rotation = to_rotation((Vec3f(0, 0, 1), block_angle))
    # Rotate the arm around the x-axis which is its axis of rotation
    arm_rotation   = to_rotation((Vec3f(1, 0, 0), arm_angle))
    # Rotate the plots to reflect the current state of the pendulum.
    rotate!(block_plot, block_rotation)
    rotate!(arm_plot, block_rotation * arm_rotation)
end

# ## Control code
# From here, all the code is adapted from the `swingup.jl` example.
# The main difference is that we use the `@periodically_yielding` macro
# allows the graphics context to be updated in the main thread,
# while the control loop runs in the background.
normalize_angles(x::Number) = mod(x, 2pi)
normalize_angles(x::AbstractVector) = SA[(x[1]), normalize_angles(x[2]), x[3], x[4]]

# Note the new kwarg here - the syntax
# to update a Ref and an Observable is the same
# so they are interchangeable, and you can pass 
# either an Observable or a Ref to `force`.
function swingup(process; force = Ref(80.0), Tf = 10, verbose=true, stab=true, umax=2.0)
    Ts = process.Ts
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float64}}(undef, 0)
    sizehint!(data, N)

    simulation = processtype(process) isa SimulatedProcess

    if simulation
        u0 = 0.0
    else
        u0 = 0.5QuanserInterface.go_home(process)
        @show u0
    end
    y = QuanserInterface.measure(process)
    if verbose && !simulation
        @info "Starting $(simulation ? "simulation" : "experiment") from y: $y, waiting for your input..."
        # readline()
    end
    yo = @SVector zeros(2)
    dyf = @SVector zeros(2)
    L =  SA[-2.8515070942708687 -24.415803244034326 -0.9920297324372649 -1.9975963404759338]
    # L = [-7.410199310542298 -36.40730995983665 -2.0632501290782095 -3.149033572767301] # State-feedback gain Ts = 0.01

    try
        # GC.gc()
        GC.enable(false)
        t_start = time()
        u = [0.0]
        oob = 0

        for i = 1:N
            @periodically_yielding Ts begin 
                t = simulation ? (i-1)*Ts : time() - t_start
                y = QuanserInterface.measure(process)
                dy = (y - yo) ./ Ts
                dyf = @. 0.5dyf + 0.5dy
                xh = [y; dyf]
                xhn = SA[xh[1], normalize_angles(xh[2]), xh[3], xh[4]]
                r = rr[]
                if !(-deg2rad(110) <= y[1] <= deg2rad(110))
                    u = SA[-0.5*y[1]]
                    verbose && @warn "Correcting"
                    control(process, Vector(u .+ u0))
                    oob += 20
                    if oob > 1000
                        verbose && @error "Out of bounds"
                        QuanserInterface.go_home(process; th = 15)
                        continue
                    end
                else
                    oob = max(0, oob-1)
                    if stab && abs(normalize_angles(y[2]) - pi) < 0.40
                        verbose && @info "stabilizing"
                        # if floor(Int, 2t) % 2 == 0
                        #     r[1] = -deg2rad(20)
                        # else
                        #     r[1] = deg2rad(20)
                        # end
                        # r[1] = deg2rad(20)*sin(2pi*t/1)

                        u = clamp.(L*(r - xhn), -10, 10)
                    else
                        # xhn = (process.x) # Try with correct state if in simulation
                        α = y[2] - pi
                        αr = r[2] - pi
                        α̇ = xh[4]
                        E = energy(α, α̇)
                        # NOTE: this is where you get the force
                        uE = force[] * (E - energy(αr,0))*sign(α̇*cos(α))
                        u = SA[clamp(uE - 0.2*y[1], -umax, umax)]
                    end
                    control(process, Vector(u))
                end
                verbose && @info "t = $(round(t, digits=3)), u = $(u[]), xh = $xh"
                log = [t; y; xh; u]
                push!(data, log)
                yo = y
            end
        end
    catch e
        @error "Terminating" e
        # rethrow()
    finally
        control(process, [0.0])
        GC.enable(true)
        # GC.gc()
    end

    reduce(hcat, data)
end
##
# home!(process, 38)
##

if processtype(process) isa SimulatedProcess
    process.x = 0*process.x
elseif abs(y[2]) > 0.8 || !(-2.5 < y[1] < 2.5)
    @info "Auto homing"
    autohome!(process)
end
# Spawn the control loop in a new thread.
# This allows graphics updates to happen on the main thread,
# while the control loop runs separately, but in the same process.
task = Threads.@spawn swingup(process; force = force, Tf = 200, verbose = false)

# To interrupt:
# Base.throwto(task, InterruptException())