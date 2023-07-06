module QuanserInterface

export QubeServo, QubeServoPendulum, QubeServoPendulumSimulator
export home_arm!, home_pend!, home!

using StaticArrays
using PythonCall
using HardwareAbstractions
using ControlSystemsBase
import HardwareAbstractions as hw
import HardwareAbstractions: control, measure, inputrange, outputrange, isstable, isasstable, sampletime, bias, initialize, finalize, processtype, ninputs, noutputs, nstates


const HIL = Ref(Py(nothing))
const card = Ref(Py(nothing))

function pendulum_parameters()
    ## Motor
    # Resistance
    Rm = 8.4
    # Current-torque (N-m/A)
    kt = 0.042
    # Back-emf constant (V-s/rad)
    km = 0.042
    #
    ## Rotary Arm
    # Mass (kg)
    mr = 0.095
    # Total length (m)
    r = 0.085
    # Moment of inertia about pivot (kg-m^2)
    Jr = mr*r^2/3
    # Equivalent Viscous Damping Coefficient (N-m-s/rad)
    br = 1e-3 # damping tuned heuristically to match QUBE-Sero 2 response
    #
    ## Pendulum Link
    # Mass (kg)
    mp = 0.024
    # Total length (m)
    Lp = 0.129
    # Pendulum center of mass (m)
    l = Lp/2
    # Moment of inertia about pivot (kg-m^2)
    Jp = mp*Lp^2/3
    # Equivalent Viscous Damping Coefficient (N-m-s/rad)
    bp = 5e-5 # damping tuned heuristically to match QUBE-Sero 2 response
    # Gravity Constant
    g = 9.81
    (; Rm, kt, km, mr, r, Jr, br, mp, Lp, l, Jp, bp, g)
end

function linearized_pendulum(p = pendulum_parameters())
    (; Rm, kt, km, mr, r, Jr, br, mp, Lp, l, Jp, bp, g) = p
    # Find Total Inertia
    Jt = Jr*Jp - mp^2*r^2*l^2
    # 
    # State Space Representation
    A = [0 0 1 0;
        0 0 0 1;
        0 mp^2*l^2*r*g/Jt  -br*Jp/Jt   -mp*l*r*bp/Jt 
        0  mp*g*l*Jr/Jt    -mp*l*r*br/Jt   -Jr*bp/Jt]
    #
    B = [0; 0; Jp/Jt; mp*l*r/Jt]
    C = [1 0 0 0; 0 -1 0 0]
    D = zeros(2,1)
    # 
    # Add actuator dynamics
    A[3,3] -= km*km/Rm*B[3]
    A[4,3] -= km*km/Rm*B[4]
    B = km * B / Rm
    ss(A, B, C, D)
end


function check(result)
    res = pyconvert(Union{Int, Nothing}, result)
    success = res === nothing
    success || @error("Failed: $result")
    success
end

abstract type QuanserBackend end

struct PythonBackend <: QuanserBackend
    card::Py
    digital_channel_write::Vector{UInt32}
    encoder_channel::Vector{UInt32}
    analog_channel_write::Vector{UInt32}
    analog_channel_read::Vector{UInt32}
    encoder_read_buffer::Vector{Int32}
    analog_read_buffer::Vector{Int32}
end

function measure(p::PythonBackend)
    if !isempty(length(p.encoder_channel))
        result = p.card.read_encoder(p.encoder_channel, length(p.encoder_channel), p.encoder_read_buffer)
        check(result)
    end
    if !isempty(p.analog_channel_read)
        result = p.card.read_analog(p.analog_channel_read, length(p.analog_channel_read), p.analog_read_buffer)
        check(result)
    end
    [p.encoder_read_buffer; p.analog_read_buffer]
end

function control(p::PythonBackend, u::Vector{Float64})
    result = p.card.write_analog(p.analog_channel_write, length(p.analog_channel_write), u)
    check(result)
    u
end

function initialize(p::PythonBackend)
    enable = [UInt32(1)]
    result = p.card.write_digital(p.digital_channel_write, 1, enable)
    check(result)
end

finalize(p::PythonBackend) = p.card.close()


function load_default_backend(; 
    digital_channel_write::Vector{UInt32},
    encoder_channel::Vector{UInt32},
    analog_channel_write::Vector{UInt32},
    analog_channel_read::Vector{UInt32},
    encoder_read_buffer::Vector{Int32},
    analog_read_buffer::Vector{Int32},
)
    if HIL[] === Py(nothing)
        @info "Loading quanser Python module"
        sys = pyimport("sys")
        sys.path.append("/home/fredrikb/quanser")
        HIL[] = pyimport("quanser.hardware" => "HIL")
    end
    if card[] !== Py(nothing)
        # If already loaded, close first
        @info "Closing previous HIL card"
        card[].close()
        sleep(0.1)
    end
    @info "Loading HIL card"
    card[] = try_twice(()->HIL[]("qube_servo3_usb", "0"))
    PythonBackend(
        card[],
        digital_channel_write,
        encoder_channel,
        analog_channel_write,
        analog_channel_read,
        encoder_read_buffer,
        analog_read_buffer,
    )
end

"""
    try_twice(f)
Ja ja om du trugar
"""
function try_twice(f)
    try
        f()
    catch e
        if e isa PyException occursin("HILError: -1073", string(e))
            sleep(0.2)
            try
                return f()
            catch
                @error "Failed twice, is the device power off, or did you forget to call finalize?"
                rethrow()
            end
        else
            rethrow()
        end
    end
end


# ==============================================================================
## QubeServo
# ==============================================================================

abstract type AbstractQubeServo <: AbstractProcess end
@kwdef struct QubeServo{B <: QuanserBackend} <: AbstractQubeServo
    Ts::Float64 = 0.01
    backend::B  = load_default_backend(
        digital_channel_write = UInt32[0],
        encoder_channel       = UInt32[0],
        analog_channel_write  = UInt32[0],
        analog_channel_read   = UInt32[],
        encoder_read_buffer   = zeros(Int32, 1),
        analog_read_buffer    = zeros(Int32, 0),
    )
end

processtype(::QubeServo) = PhysicalProcess()
control(p::AbstractQubeServo, u::Vector{Float64}) = control(p.backend, u)
control(p::AbstractQubeServo, u::Number) = control(p, [u])
ninputs(p::AbstractQubeServo)  = 1
inputrange(p::AbstractQubeServo)  = [(-10,10)]
finalize(p::AbstractQubeServo) = finalize(p.backend)
initialize(p::AbstractQubeServo) = initialize(p.backend)
sampletime(p::AbstractQubeServo)  = p.Ts
bias(p::AbstractQubeServo)        = 0.0

function measure(p::AbstractQubeServo)
    y = measure(p.backend)
    y .* 2pi / 2048 # radians
end

noutputs(p::QubeServo) = 1
nstates(p::QubeServo) = 2
outputrange(p::QubeServo) = [(-10,10)]
isstable(p::QubeServo)    = true
isasstable(p::QubeServo)  = true # Friction


# ==============================================================================
## QubeServoPendulum
# ==============================================================================

@kwdef struct QubeServoPendulum{B <: QuanserBackend} <: AbstractQubeServo
    Ts::Float64 = 0.01
    backend::B  = load_default_backend(
        digital_channel_write = UInt32[0],
        encoder_channel       = UInt32[0, 1],
        analog_channel_write  = UInt32[0],
        analog_channel_read   = UInt32[0],
        encoder_read_buffer   = zeros(Int32, 2),
        analog_read_buffer    = zeros(Int32, 0),
    )
    left_flange::Base.RefValue{Float64} = Ref(0.0)
    downward::Base.RefValue{Float64} = Ref(0.0)
end

processtype(::QubeServoPendulum) = PhysicalProcess()
noutputs(p::QubeServoPendulum) = 2
nstates(p::QubeServoPendulum) = 4
isstable(p::QubeServoPendulum)    = false
isasstable(p::QubeServoPendulum)  = false
function home_arm!(p::QubeServoPendulum)
    ybits = measure(p.backend)[1]
    y = ybits * 2pi / 2048 # radians
    @info "Homing arm"
    p.left_flange[] = y
end

function home_pend!(p::QubeServoPendulum)
    ybits = measure(p.backend)[2]
    y = ybits * 2pi / 2048 # radians
    @info "Homing pendulum"
    p.downward[] = y
end

function home!(p::QubeServoPendulum)
    home_arm!(p)
    home_pend!(p)
    nothing
end

function measure(p::QubeServoPendulum)
    ybits = measure(p.backend)
    y = ybits .* 2pi / 2048 # radians
    y[1] = y[1] - p.left_flange[] + deg2rad(137)
    y[2] = y[2] - p.downward[]
    y
end


@kwdef mutable struct QubeServoPendulumSimulator{X, F, P, D, M} <: AbstractQubeServo
    const Ts::Float64 = 0.01
    x::X = @SVector zeros(4)
    const ddyn::F = hw.rk4(furuta, Ts; supersample=10)
    p::P = pendulum_parameters()
    dynamics::D = furuta
    measurement::M = (x, u, p, t) -> SA[x[1], (x[2] - 0*pi)]
end

function furuta_parameters(; 
        m = 0.024,
        mr = 0.095,
        l = 0.129,
        M = 0.001,
        r = 0.085,
        Jp = mr*r^2/3,
        J = m*l^2/3,
        g = 9.82,
        k = 0.005, # Motor gain
    ) 
    p = (; M, l, r, J, Jp, m, g, k)
end


processtype(::QubeServoPendulumSimulator) = SimulatedProcess()

function measure(p::QubeServoPendulumSimulator)
    p.measurement(p.x, 0, p.p, 0)
end

# function furuta(x, u, p, t)
#     ϕ, θ, ϕ̇, θ̇ = x
#     M, l, r, J, Jp, m, g, k = furuta_parameters()
#     Rm = 8.4
#     km = 0.042
#     τ = km*(only(u) - km*ϕ̇) / Rm
#     α = Jp+M*l^2
#     β = J+M*r^2+m*r^2
#     γ = M*r*l
#     ϵ = l*g*(M+m/2)
#     sθ = sin(θ)
#     cθ = cos(θ)
#     C = 1/(α*β+α^2*(sθ)^2-γ^2*cθ^2)
#     scθ = sθ*cθ
#     ϕ̈ = C*(-γ*α*ϕ̇^2*sθ*cθ^2-γ*ϵ*scθ+γ*α*θ̇^2*sθ-2*α^2*θ̇*ϕ̇*scθ+α*τ)
#     θ̈ = C*((α*β+α^2*(sθ)^2)*ϕ̇^2*scθ-γ^2*θ̇^2*scθ+2*α*γ*θ̇*ϕ̇*sθ*cθ^2-γ*cθ*τ+(α*β+α^2*(sθ)^2)*ϵ/α*sθ)
#     SA[
#         ϕ̇
#         θ̇
#         ϕ̈ - 4*ϕ̇ # TODO: Tune damping
#         θ̈ - 1*θ̇
#     ]
# end

@fastmath function furuta(x, u, p, t) # Quanser equations
    θ, α, θ̇, α̇ = x
    α = pi-α

    Rm, kt, km, mr, r, Jr, br, mp, Lp, l, Jp, bp, g = p
    Lr = r

    Jr = 5.7e-5
    Jp = 3.4e-5

    Dp = 0.1*bp
    Dr = 0.1br

    # Dp = 0.0005
    # Dr = 0.0015

    τ = km*(only(u) - km*θ̇) / Rm
    
    sa = sin(α)
    ca = cos(α)

    H11 = mp*Lr^2 + 1/4*mp*Lp^2 - 1/4*mp*Lp^2*ca^2 + Jr
    H21 = 1/2*mp*Lp*Lr*ca
    H22 = Jp + 1/4*mp*Lp^2
    H = SA[
        H11 H21 # The design sheet has -H21 here which does not make sense to me, we expect the mass matrix to be symmetric. We also need + to match the linearized model from the design sheet
        H21 H22
    ]

    C = mp*Lp^2*sa*ca*θ̇
    gr = 1/2*mp*Lp*g*sa
    G = SA[
        -1/2*mp*Lp*Lr*sa*α̇^2 + τ - Dr*θ̇
        -Dp*α̇ - gr
    ]


    xdd = H \ (C * SA[-1/2*α̇; 1/4*θ̇] + G)
    SA[
        θ̇
        α̇
        xdd[1]
        xdd[2]
    ]
end

function control(p::QubeServoPendulumSimulator, u)
    p.x = p.ddyn(p.x, u, p.p, 0)
    u
end

control(p::QubeServoPendulumSimulator, u::Vector{Float64}) = @invoke control(p, u::Any)

initialize(p::QubeServoPendulumSimulator) = nothing
finalize(p::QubeServoPendulumSimulator) = nothing

"""
    go_home(process; th = 10, r = 0, Ki = 0)

Go to r using a P controller

# Arguments:
- `th`: Max control
- `r`: Reference position
- `Ki`: Integral gain (set to 0.15 or something like that if you are using the inertia disc)
"""
function go_home(process; th=5, r = 0, K = 0.8, Ki=process isa QubeServoPendulum ? 0.0 : 0.15)
    Ts = process.Ts
    count = 0
    int = 0.0
    yo = 0.0
    initialize(process)
    while true
        @periodically Ts begin
            y = measure(process)[1]
            dy = (y - yo) / Ts
            yo = y
            e = r-y
            if abs(e) < deg2rad(5) && abs(dy) < deg2rad(3)
                count += 1
                if count > 20
                    break
                end
            else
                count = 0
            end
             # V / rad
            u0 = K*e + Ki*int
            if -th <= u0 <= th
                int += Ki*e
            end
            u = clamp(u0, -th, th)
            @show u, y
            control(process, [u])
        end
    end
    # control(process, [0.0])
end

const Lup = SA[-7.8038045417791615 -38.734485788275485 -2.387482462896501 -3.285300064621874]
const Ldown = SA[8.59053220448398 -1.3750742576909472 0.7557495972343583 -0.2008266766259501]

end
