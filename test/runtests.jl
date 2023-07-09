using HardwareAbstractions
using QuanserInterface
using Test

function chirp_demo(p; 
    f0 = 0.08,
    f1 = 10,
    Tf = 28,
    gain = 0.05,
    u_max = 10.0,
)
    initialize(p)
    Ts = sampletime(p)
    N = round(Int, Tf/Ts)
    data = Vector{SVector{5, Float64}}(undef, 0)
    sizehint!(data, N)
    t_start = time()
    y_start = measure(p)[]
    try
        GC.gc()
        GC.enable(false)
        for i = 1:N
            @periodically Ts begin 
                t = time() - t_start
                y = measure(p)[] - y_start # Subtract initial position for a smoother experience
                # r = 45sin(2π*freq*t)
                r = 45*chirp(t, f0, f1, Tf; logspace=true)
                e = r - y
                u = clamp(gain*e, -u_max, u_max)
                control(p, [u])
                log = SA[t, y, u, r, e]
                push!(data, log)
            end
        end
    catch e
        @error "Shit hit the fan" e
    finally
        control(p, [0.0])
        GC.enable(true)
        GC.gc()
    end

    D = reduce(hcat, data)

    ti = 1; yi = 2; ui = 3; ri = 4; ei = 5;

    tvec = D[ti, :]
    fig = plot(tvec, D[yi, :], layout=2, lab="y")
    plot!(tvec, D[ri, :], lab="r", sp=1)
    plot!(tvec, D[ei, :], lab="e", sp=1)
    plot!(tvec, D[ui, :], lab="u", sp=2)
    finalize(p)
    (; D, fig)
end


p = QubeServo()

# show_measurements(p) do data
#     display(plot(reduce(hcat, data)'))
# end

# D, fig = chirp_demo(p; Tf = 10)
# fig

# ==============================================================================
## Pendulum
process = QubeServoPendulum(; Ts = 0.01)
psim = QubeServoPendulumSimulator(; Ts = process.Ts)
##
# ==============================================================================
using ControlSystemsBase, QuanserInterface, RobustAndOptimalControl
measure = QuanserInterface.measure
normalize_angles_pi(x::Number) = mod(x+pi, 2pi)-pi
normalize_angles_pi(x::AbstractVector) = SA[normalize_angles_pi(x[1]), normalize_angles_pi(x[2]), x[3], x[4]]


# sys2 = QuanserInterface.linearized_pendulum()
sys = let (A,B) = MPC.linearize(psim.dynamics, [0, pi, 0, 0], [0], psim.p, 0)
    let (C,D) = MPC.linearize(psim.measurement, [0, pi, 0, 0], [0], psim.p, 0)
        ss(A,B,C,D)
    end
end
# sysaug = add_measurement_disturbance(sys, [-1e-8;;], [0; 1;;])
# sysaug = add_low_frequency_disturbance(sys, 1, ϵ = 1e-8)
sysd = c2d(sys, psim.Ts)

Q1 = Diagonal([1000, 10, 1, 1])
Q2 = 10I(1)


R1b = [
    zeros(2, 4)
    [zeros(2, 2) diagm([1, 1])]
]
R1bd = c2d(ss(sys.A, R1b, I, 0), psim.Ts).B
R1 = Symmetric(1000R1bd*R1bd')
R2 = 0.001I(2)


R1 = kron(LowLevelParticleFilters.double_integrator_covariance(Ts_fast, 1000), I(2)) + 1e-9I
R2 = 2pi/2048 * diagm([0.1, 0.1])

syse = ExtendedStateSpace(sysd, C1=I, B1=I)



# using Ipopt, OptimizationMOI, Optimization
# MOI = OptimizationMOI.MOI
# using ImplicitDifferentiation

# function triangular(x)
#     m = length(x)
#     n = round(Int, sqrt(2m-1))
#     T = zeros(eltype(x), n, n)
#     k = 1
#     for i = 1:n, j = i:n
#         T[i,j] = x[k]
#         k += 1
#     end
#     T
# end
# invtriangular(T) = [T[i,j] for i = 1:size(T,1) for j = i:size(T,1)]

# "A helper function that simulates the closed-loop system"
# function sim(S, PS; Tf)
#     res1 = step(S, Tf)   # Simulate S
#     res2 = step(PS, Tf)  # Simulate PS
#     res1, res2
# end
# # function sim(G)
# #     Gd = c2d(G, 0.1, :tustin)   # Discretize the system
# #     res1 = step(Gd[1, 1], 0:0.1:15) # Simulate S
# #     res2 = step(Gd[1, 2], 0:0.1:15) # Simulate PS
# #     res1, res2
# # end

# function systemslqr(params::AbstractVector, P)
#     n2 = length(params) ÷ 2
#     Qchol = triangular(params[1:n2])
#     Rchol = triangular(params[n2+1:2n2])
#     et = eltype(params)
#     Q = Qchol'Qchol
#     R = Rchol'Rchol
#     L = lqr(P, Q, et.(Q2)) # It's important that the last matrix has the correct type
#     K = kalman(P, R, et.(R2))
#     C = observer_controller(P, L, K)
#     S, PS, CS, T = gangoffour(P, C, minimal=false) # [S PS; CS T]
#     S = input_sensitivity(P, C)
#     C, S, PS, CS, T, Q, R
# end
# function optimize_lqq(P, Q1, Q2, R1, R2; Tf = 2, CSweight = 0.0001, Msc=5.0)
#     Ω = ControlSystemsBase._default_freq_vector(P, Val(:bode))
    

#     "The cost function to optimize"
#     function cost(params::AbstractVector, P) 
#          # Noise amplification penalty
#         C, S, PS, CS, T = systemslqr(params, P)
#         res1, res2 = sim(S, PS; Tf)
#         R, _ = sigma(CS, Ω)
#         CSR = sum(R .*= Ω')               # frequency-weighted noise sensitivity
#         perf = mean(abs2, res1.y .*= res1.t') + mean(abs2, res2.y .*= res2.t')
#         return perf + CSweight * CSR # Blend objectives together
#     end

#     @views function constraints(res, params::AbstractVector, P)
#         C, S, PS, CS, T = systemslqr(params, P)
#         # res .= hinfnorm(S, tol=1e-7)[1]
#         res .= maximum(sigma(S, Ω)[1])
#         nothing
#     end
    

#     params = [invtriangular(cholesky(Q1).U); invtriangular(cholesky(R1).U)]
    
#     fopt = OptimizationFunction(cost, Optimization.AutoFiniteDiff(); cons=constraints)
#     prob = OptimizationProblem(fopt, params, P;
#         lb    = 0.5*abs.(params),
#         ub    = 2*abs.(params),
#         ucons = fill(float(Msc), 1),
#         lcons = fill(-Inf, 1),
#     )
#     solver = Ipopt.Optimizer()
#     MOI.set(solver, MOI.RawOptimizerAttribute("print_level"), 5)
#     MOI.set(solver, MOI.RawOptimizerAttribute("tol"), 1e-3)
#     MOI.set(solver, MOI.RawOptimizerAttribute("acceptable_tol"), 1e-1)
#     MOI.set(solver, MOI.RawOptimizerAttribute("max_iter"), 1000)
#     MOI.set(solver, MOI.RawOptimizerAttribute("max_wall_time"), 20.0)
#     MOI.set(solver, MOI.RawOptimizerAttribute("constr_viol_tol"), 1e-2)
#     MOI.set(solver, MOI.RawOptimizerAttribute("acceptable_constr_viol_tol"), 0.1)
#     MOI.set(solver, MOI.RawOptimizerAttribute("acceptable_iter"), 5)
#     MOI.set(solver, MOI.RawOptimizerAttribute("acceptable_obj_change_tol"), 1e-2)
#     MOI.set(solver, MOI.RawOptimizerAttribute("hessian_approximation"), "limited-memory")
#     res = solve(prob, solver)
#     C, S, PS, CS, T, Q, R = systemslqr(res.u, P)
#     (; res, C, S, PS, CS, T, Msc, Q, R)
# end

# res = optimize_lqq(sysd, Q1, Q2, R1 + 1e-8I, R2; Tf = 2, CSweight = 0.0005, Msc=2.5)
# P = sysd
# (; C, S, PS, CS, T, Msc, Q, R) = res
# r1, r2 = sim(S, PS; Tf= 2)
# lab = "Optimized"
# f1 = plot(
#     r1;
#     title     = "Time response",
#     lab       = lab * " \$r → e\$",
#     legend    = :bottomright,
#     fillalpha = 0.05,
#     linealpha = 0.8,
#     c         = 1,
#     layout    = (2,1), sp=1,
# )
# plot!(
#     r2;
#     lab       = lab * " \$d → e\$",
#     fillalpha = 0.05,
#     linealpha = 0.8,
#     c         = 1,
#     sp        = 2
# )
# f2 = gangoffourplot(P, C, title="", xlabel="", Ms_lines=nothing)
# Si = input_sensitivity(P, C)
# bodeplot!(f2, Si, sp=1, plotphase=false)
# hline!([Msc]; l=:dashdot, c=1, lab="Constraint", ylims=(9e-2, Inf))
# f3 = nyquistplot(C*P; Ms_circles=Msc, ylims=(-2.1, 1.1), xlims=(-2.1, 1.2), lab)
# plot(f1, f2, f3; layout=(1, 3), size=(1800, 700), bottommargin=2Plots.mm)
# #
# Q1 = Q
# R1 = R

##
using ControlSystemsBase; Discrete = ControlSystemsBase.Discrete
function observer(sys::AbstractStateSpace{<:Discrete}, K)
    A, B, C, D = ssdata(sys)
    ss(A - K*C, [B K], I, 0, sys.timeevol)
end

prob = LQGProblem(syse, Q1, Q2, R1, R2, qQ=0, qR = 0.0)
L = lqr(prob)
K = kalman(prob)

L = SMatrix{size(L)...}(L)
K = SMatrix{size(K)...}(K)
obs = observer(sysd, K)
xr = SA[0, pi, 0, 0]
x0 = SA[0, 0.0, 0, 0.0]

C = observer_controller(sysd, L, K)
Cgmf, γ, gmfinfo = glover_mcfarlane(sysd, W1=pid(50,0.02; sysd.Ts))
Ms = hinfnorm(input_sensitivity(sysd, C))[1]
f1 = gangoffourplot(sysd, [C, Cgmf])
Li = C*sysd
Ligmf = Cgmf*sysd
f2 = nyquistplot([Li, Ligmf], Ms_circles=[2, Ms], ylims=(-2, 2), xlims=(-3,2), lab="Ms = $Ms")
f3 = marginplot([Li, Ligmf])
vline!([27.2], sp=1)
plot(f1, f2, f3)

##
velfilt1 = c2d(ss(tf([50, 0], [1, 50])), p.Ts, :tustin) |> SysFilter
velfilt2 = c2d(ss(tf([50, 0], [1, 50])), p.Ts, :tustin) |> SysFilter
gmffilt = SysFilter(Cgmf)
#

function energy(θ, θ̇)
    mp = 0.024
    Lp = 0.129
    l = Lp/2
    Jp = mp*Lp^2/3
    Jp_cm = mp*Lp^2/12
    g = 9.81
    θ = θ + pi
    E = 1/2*Jp_cm*θ̇^2 + mp*g*l*(1-cos(θ))
end

function controller(u, y, obsfilter)
    θ = y[2] - pi
    ϕ̇ = velfilt1(y[1])[]
    θ̇ = velfilt2(θ)[]
    θ = normalize_angles_pi(θ)
    xh = obsfilter([u; y[1]; θ])
    # ugmf = gmffilt([y[1]; θ])
    # xh = [y[1]; θ; ϕ̇; θ̇]
    E = energy(θ, θ̇)
    if abs(θ) < 0.3 # Use stabilizing controller
        u = -L*xh
        # u =- ugmf
        u, xh
    else
        u = 0*[80*(E - energy(0,0))*sign(θ̇*cos(θ)) - 0.1*y[1]] # Bias slightly towards center
        @. u = clamp(u, -4, 4)
    end
    u, xh, E
end


function balance_demo(p; 
    u_max = 2.0,
    Tf = 10,
    controller = controller,
    obsfilter = SysFilter(obs, copy(x0)),
)

    velfilt1.state .= 0
    velfilt2.state .= 0
    # initialize(p)
    Ts = p.Ts
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float64}}(undef, 0)
    sizehint!(data, N)
    
    t_start = time()
    # y_start .* [1, 1]
    simulation = processtype(p) isa SimulatedProcess
    simulation || QuanserInterface.go_home(p; K = 0.2, Ki=0.5, Kf=0.0)
    y_start = measure(p)
    try
        GC.gc()
        GC.enable(false)
        u = [0.0]
        for i = 1:N
            @periodically_yielding Ts begin 
                t = time() - t_start
                y = measure(p) - y_start # Subtract initial position for a smoother experience
                u, xh, E = controller(u, y, obsfilter)
                u = @. clamp(u, -u_max, u_max)
                @show u
                control(p, Vector(u))
                log = [t; y; xh; u; E]
                push!(data, log)
            end
        end
    catch e
        @error "Shit hit the fan" e
        # rethrow()
    finally
        control(p, [0.0])
        GC.enable(true)
        GC.gc()
    end

    D = reduce(hcat, data)
end
##0
D = balance_demo(process; u_max=8, Tf = 20)

function plotD(D)
    tvec = D[1, :]
    y = D[2:3, :]'
    y[:, 2] .-= pi
    y[:, 2] .*= -1
    xh = D[4:7, :]'
    u = D[8, :]
    E = D[9, :]
    plot(tvec, xh, layout=6)
    plot!(tvec, y, sp=[1 2])
    plot!(tvec, u, sp=5, lab="u")
    plot!(tvec, E, sp=6, lab="E")
end
plotD(D)
##
x0 = zeros(sysd.nx)
obsfilter = SysFilter(obs, copy(x0))
show_measurements(p) do data
    y = data[end]
    @show xh = obsfilter([0; y])
    display(plot(reduce(hcat, data)'))
end

##



# writedlm("/tmp/qubeservo.csv", ["t" "y" "r" "e" "u"; D'], ',')


# using ControlSystemIdentification, ControlSystemsBase, Statistics, Plots
# d = iddata(D[yi, :], D[ui, :], median(diff(tvec)))
# d = detrend(d)
# plot(d)
# model, _ = newpem(d, 2, zeroD=true)
# model = arx(d, 2, 2)
# model = subspaceid(d, 2)
# plot(
#     bodeplot(model, hz=true),
#     predplot(model, d),
#     simplot(model, d)
# )
##

