__precompile__(false)
module QuanserInterface

export QubeServo, QubeServoPendulum, QubeServoPendulumSimulator4


using StaticArrays
using PythonCall
using HardwareAbstractions
using ControlSystemsBase
import HardwareAbstractions as hw
import HardwareAbstractions: control, measure, num_inputs, num_outputs, inputrange, outputrange, isstable, isasstable, sampletime, bias, initialize, finalize, processtype

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
    sys = pyimport("sys")
    sys.path.append("/home/fredrikb/quanser")
    HIL = pyimport("quanser.hardware" => "HIL")
    PythonBackend(
        try_twice(()->HIL("qube_servo3_usb", "0")),
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
end

processtype(::QubeServoPendulum) = PhysicalProcess()
noutputs(p::QubeServo) = 2
nstates(p::QubeServo) = 4
outputrange(p::QubeServo) = [(-10,10), (-pi/2, pi/2)]
isstable(p::QubeServo)    = false
isasstable(p::QubeServo)  = false


@kwdef mutable struct QubeServoPendulumSimulator4{X, F, P, D, M} <: AbstractQubeServo
    const Ts::Float64 = 0.01
    x::X = @SVector zeros(4)
    const ddyn::F = hw.rk4(furuta, Ts; supersample=10)
    p::P = furuta_parameters()
    dynamics::D = furuta
    measurement::M = (x, u, p, t) -> SA[x[1], x[2] - pi]
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


processtype(::QubeServoPendulumSimulator4) = SimulatedProcess()

function measure(p::QubeServoPendulumSimulator4)
    p.measurement(p.x)
end

function furuta(x, u, p, t)
    ϕ, θ, ϕ̇, θ̇ = x
    θ = θ - pi
    M, l, r, J, Jp, m, g, k = p
    τ = k*only(u)
    α = Jp+M*l^2
    β = J+M*r^2+m*r^2
    γ = M*r*l
    ϵ = l*g*(M+m/2)
    sθ = sin(θ)
    cθ = cos(θ)
    C = 1/(α*β+α^2*(sθ)^2-γ^2*cθ^2)
    scθ = sθ*cθ
    θ̈ = C*((α*β+α^2*(sθ)^2)*ϕ̇^2*scθ-γ^2*θ̇^2*scθ+2*α*γ*θ̇*ϕ̇*sθ*cθ^2-γ*cθ*τ+(α*β+α^2*(sθ)^2)*ϵ/α*sθ)
    ϕ̈ = C*(-γ*α*ϕ̇^2*sθ*(cos(θ̇))^2-γ*ϵ*scθ+γ*α*θ̇^2*sθ-2*α^2*θ̇*ϕ̇*scθ+α*τ)
    SA[
        ϕ̇
        θ̇
        ϕ̈ - 4*ϕ̇ # TODO: Tune damping
        θ̈ - 100*θ̇
    ]
end

function control(p::QubeServoPendulumSimulator4, u)
    x = p.x
    xp = p.ddyn(x, u, p.p, 0)
    p.x = xp
    u
end

control(p::QubeServoPendulumSimulator4, u::Vector{Float64}) = @invoke control(p, u::Any)

initialize(p::QubeServoPendulumSimulator4) = nothing
finalize(p::QubeServoPendulumSimulator4) = nothing

end
