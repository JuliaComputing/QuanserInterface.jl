#=
Pendulum swingup controller reimplemented using the SynchJulia synchronous dataflow DSL.
The controller is decomposed into separate @node functions:
- velocity_estimator: finite-difference + exponential smoothing
- energy_swingup: energy-based swingup controller
- lqr_stabilizer: LQR state-feedback stabilization
- swingup_node: top-level mode-switching orchestrator

Runs on the simulated process only.
=#
cd(@__DIR__)
using Pkg; Pkg.activate("..")
using QuanserInterface
using HardwareAbstractions
using SynchCompiler, SynchRuntime
using StaticArrays

# ==============================================================================
## FFI helper functions (called from @node via FFI)
# ==============================================================================

function energy(α::Float64, α̇::Float64)::Float64
    mp = 0.024
    Lp = 0.129
    g = 9.81
    l = Lp / 2
    Jp_cm = mp * Lp^2 / 12
    return 0.5 * Jp_cm * α̇^2 + mp * g * l * (1 + cos(α))
end


# ==============================================================================
## SynchJulia nodes
# ==============================================================================

# Velocity estimation: finite-difference differentiation + first-order exponential filter
@node function velocityestimator(y::Float64, ts::Float64)::(dyfiltered::Float64)
    yo = pre(y; init=0.0)
    dy = (y - yo) / ts
    prevdyf = pre(dyfiltered; init=0.0)
    dyfiltered = 0.5 * prevdyf + 0.5 * dy
end

# Energy-based swingup controller
@node function energyswingup(θ::Float64, α::Float64, α̇::Float64, umax::Float64)::(u::Float64)
    αshifted = α - 3.141592653589793
    e = energy(αshifted, α̇)
    eref = energy(0.0, 0.0)
    ue = 80.0 * (e - eref) * sign(α̇ * cos(αshifted))
    u = clamp(ue - 0.2 * θ, -umax, umax)
end

# LQR stabilization controller (gains designed for ts=0.01)
@node function lqrstabilizer(θ::Float64, αnorm::Float64, dθ::Float64, dα::Float64)::(u::Float64)
    e1 = 0.0 - θ
    e2 = 3.141592653589793 - αnorm
    e3 = 0.0 - dθ
    e4 = 0.0 - dα
    uraw = -7.410199310542298 * e1 + -36.40730995983665 * e2 + -2.0632501290782095 * e3 + -3.149033572767301 * e4
    u = clamp(uraw, -10.0, 10.0)
end

# Top-level swingup controller: mode switching between OOB correction, LQR, and energy swingup
@node function swingupnode(θ::Float64, α::Float64, ts::Float64, umax::Float64)::(u::Float64)
    # Velocity estimation (each call site gets independent state)
    dθ = velocityestimator(θ, ts)
    dα = velocityestimator(α, ts)

    # Normalize pendulum angle to [0, 2π)
    αnorm = mod(α, 2pi)

    # Mode conditions
    ooblimit = deg2rad(110)
    neartop = abs(αnorm - 3.141592653589793) < 0.40
    outofbounds = (θ > ooblimit) || (θ < -ooblimit)

    # Compute control for each mode
    uoob = -0.5 * θ
    ulqr = lqrstabilizer(θ, αnorm, dθ, dα)
    uswingup = energyswingup(θ, α, dα, umax)

    # Select active mode (priority: OOB > LQR > swingup)
    u = if outofbounds
        uoob
    else
        if neartop
            ulqr
        else
            uswingup
        end
    end
end

# ==============================================================================
## Build and simulate
# ==============================================================================

exe = build(swingupnode, (Float64, Float64, Float64, Float64))

Ts = 0.01
process = QuanserInterface.QubeServoPendulumSimulator(; Ts, p = QuanserInterface.pendulum_parameters(true))

Tf = 15.0
N = round(Int, Tf / Ts)
data = Vector{Vector{Float64}}(undef, 0)
sizehint!(data, N)

for i in 1:N
    y = QuanserInterface.measure(process)
    θ, α = y[1], y[2]

    result = step!(exe, θ, α, Ts, 2.0)
    u = result.u

    control(process, [u])

    t = (i - 1) * Ts
    push!(data, [t, θ, α, u])
end
control(process, [0.0])

D = reduce(hcat, data)

# Verify: pendulum should be near [0, π]
using LinearAlgebra
final_state = D[2:3, end]
@info "Final state: θ=$(final_state[1]), α=$(final_state[2]), target: [0, ±π]"

# ==============================================================================
## Plot results
# ==============================================================================
using Plots

tvec = D[1, :]
θvec = D[2, :]
αvec = D[3, :]
uvec = D[4, :]

plot(
    plot(tvec, θvec, lab="arm θ", ylabel="rad", framestyle=:zerolines),
    plot(tvec, αvec, lab="pend α", ylabel="rad", framestyle=:zerolines,
         legend=:right),
    plot(tvec, uvec, lab="u", ylabel="V", xlabel="t [s]", framestyle=:zerolines),
    layout=(3, 1), size=(800, 600), link=:x
)
hline!([π -π], sp=2, lab="", l=(:black, :dash))
