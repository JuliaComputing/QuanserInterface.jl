
"""
    pendulum_parameters(optimized = false)

Return a named tuple with the default or `optimized` parameters. The optimized parameters have been estimated using maximum-likelihood estimation with an `LowLevelParticleFilters.UnscentedKalmanFilter`, see example `pendulum_identification.jl`.
"""
function pendulum_parameters(optimized = false)
    if optimized
        (Rm = 3.7282272826905714, kt = 0.042, km = 0.061043535870362514, mr = 0.095, r = 0.0770658109812201, Jr = 1.112300869775737e-5, br = 4.8938205931891925e-5, mp = 0.11572907399113673, Lp = 0.08108254471783813, l = 0.0645, Jp = 0.00019808351489259391, bp = 3.250070828840098e-6, g = 9.81)
    else
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
        br = 0.05*1e-3 # damping tuned heuristically to match QUBE-Sero 2 response
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
        bp = 0.05*5e-5 # damping tuned heuristically to match QUBE-Sero 2 response
        # Gravity Constant
        g = 9.81
        (; Rm, kt, km, mr, r, Jr, br, mp, Lp, l, Jp, bp, g)
    end
end

function optimized_UKF(psim)
    psim.Ts == 0.01 || error("The UKF was estimated for Ts = 0.01, to estimate parameters for a different sample rate, collect some data and make use of `examples/pendulum_identification.jl`")
    R1 = kron(llpf.double_integrator_covariance(Ts, 1), diagm([556, 965])) + 1e-9I
    p = pendulum_parameters(optimized=true)
    R2 = 2pi/2048 * I(2)
    d0 = llpf.MvNormal(psim.x, 10R1)
    kfoptRf = llpf.UnscentedKalmanFilter(psim.ddyn, psim.measurement, R1, R2, d0; ny, nu, p)
end

# function linearized_pendulum(p = pendulum_parameters())
#     (; Rm, kt, km, mr, r, Jr, br, mp, Lp, l, Jp, bp, g) = p
#     # Find Total Inertia
#     Jt = Jr*Jp - mp^2*r^2*l^2
#     # 
#     # State Space Representation
#     A = [0 0 1 0;
#         0 0 0 1;
#         0 mp^2*l^2*r*g/Jt  -br*Jp/Jt   -mp*l*r*bp/Jt 
#         0  mp*g*l*Jr/Jt    -mp*l*r*br/Jt   -Jr*bp/Jt]
#     #
#     B = [0; 0; Jp/Jt; mp*l*r/Jt]
#     C = [1 0 0 0; 0 -1 0 0]
#     D = zeros(2,1)
#     # 
#     # Add actuator dynamics
#     A[3,3] -= km*km/Rm*B[3]
#     A[4,3] -= km*km/Rm*B[4]
#     B = km * B / Rm
#     ss(A, B, C, D)
# end


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
function home_arm!(p::QubeServoPendulum, ang=137)
    control(p, [0.0])
    sleep(0.1)
    ybits = measure(p.backend)[1]
    y = ybits * 2pi / 2048 # radians
    @info "Homing arm"
    p.left_flange[] = y + deg2rad(137-ang)
end

function home_pend!(p::QubeServoPendulum)
    ybits = measure(p.backend)[2]
    y = ybits * 2pi / 2048 # radians
    @info "Homing pendulum"
    p.downward[] = y
end

function home!(p::QubeServoPendulum, args...)
    home_arm!(p, args...)
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
    measurement::M = (x, u, p, t) -> SA[x[1], x[2]]
end

processtype(::QubeServoPendulumSimulator) = SimulatedProcess()

function measure(p::QubeServoPendulumSimulator)
    p.measurement(p.x, 0, p.p, 0)
end

function furuta(x, u, p, t) # Quanser equations
    θ, α0, θ̇, α̇ = x
    α = float(pi)-α0

    Rm, kt, km, mr, r, Jr, br, mp, Lp, l, Jp, bp, g = p
    Lr = r

    # Jr = 5.7e-5
    # Jp = 3.4e-5

    Dp = bp
    Dr = br

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

    if eltype(x) <: Real
        xdd = H \ (C * SA[-1/2*α̇; 1/4*θ̇] + G)
    else
        den = (H[1, 1]*H[2, 2] - H[1, 2]*H[2, 1])
        xdd = [H[2,2] -H[1,2]; -H[2,1] H[1,1]] * (C * SA[-1/2*α̇; 1/4*θ̇] + G)  ./ den
    end
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
function go_home(process; th=5, r = 0, K = 0.2, Ki=0.15, Kf = 0.1)
    Ts = process.Ts
    count = 0
    int = 0.0
    yo = 0.0
    initialize(process)
    local u
    i = 0
    while true
        @periodically Ts begin
            i += 1
            y = measure(process)[1]
            dy = (y - yo) / Ts
            yo = y
            e = r-y
            if abs(e) < deg2rad(10) && abs(dy) < deg2rad(4)
                count += 1
                if count > 20
                    break
                end
            else
                count = 0
            end
             # V / rad
            u0 = K*e + int + Kf*sign(e) # friction compensation
            if -th <= u0 <= th
                int += Ki*e*Ts
            end
            u = clamp(u0, -th, th)
            @show u, y
            control(process, [u + (i % 2 == 0 ? 0.01 : -0.01)]) # add dither
        end
    end
    u
end

const Lup = SA[-7.8038045417791615 -38.734485788275485 -2.387482462896501 -3.285300064621874]
const Ldown = SA[8.59053220448398 -1.3750742576909472 0.7557495972343583 -0.2008266766259501]

energy(x::AbstractVector) = energy(x[2], x[4])
function energy(α, α̇)
    mp = 0.024
    Lp = 0.129
    g = 9.81
    l = Lp/2
    Jp = mp*Lp^2/3
    Jp_cm = mp*Lp^2/12
    E = 1/2*Jp_cm*α̇^2 + mp*g*l*(1+cos(α))
end
