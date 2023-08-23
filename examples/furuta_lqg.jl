#=
Design al LQG controller for the Furuta pendulum in the upright position (xr[2] = pi)
=#

using HardwareAbstractions, QuanserInterface, ControlSystemsBase, RobustAndOptimalControl, LowLevelParticleFilters, StaticArrays, Plots
using LinearAlgebra
# ==============================================================================
## Pendulum
Ts = 0.005
# process = QubeServoPendulum(; Ts)
psim = QubeServoPendulumSimulator(; Ts, p = QuanserInterface.pendulum_parameters(true))
xr = SA[0, pi, 0, 0]
x0 = SA[0, 0.0, 0, 0.0]
##
# ==============================================================================
normalize_angles_pi(x::Number) = mod(x+pi, 2pi)-pi
normalize_angles_pi(x::AbstractVector) = SA[normalize_angles_pi(x[1]), normalize_angles_pi(x[2]), x[3], x[4]]

function linearize_pendulum(xr)
    A, B = ControlSystemsBase.linearize(psim.dynamics, xr, [0], psim.p, 0)
    C, D = ControlSystemsBase.linearize(psim.measurement, xr, [0], psim.p, 0)
    ss(A,B,C,D)
end
sys = linearize_pendulum(xr)
# sysaug = add_measurement_disturbance(sys, [-1e-8;;], [0; 1;;])
# sysaug = add_low_frequency_disturbance(sys, 1, ϵ = 1e-8)
sysd = c2d(sys, Ts)

Q1 = Diagonal([1000, 10, 1, 1])
Q2 = 10I(1)

R1 = kron(LowLevelParticleFilters.double_integrator_covariance(Ts, 1000), I(2)) + 1e-9I
R2 = 2pi/2048 * diagm([0.1, 0.1])

syse = ExtendedStateSpace(sysd, C1=I, B1=I)

# include("optimize_lqg.jl") # Optionally perform optimization of Q1 and R1

direct = true
prob = LQGProblem(syse, Q1, Q2, R1, R2, qQ=0, qR = 0.0)
L = lqr(prob)
K = kalman(prob; direct)

L = SMatrix{size(L)...}(L)
K = SMatrix{size(K)...}(K)
# obs = observer_predictor(sysd, K, output_state=true)
obs = observer_filter(sysd, K, output_state=true)


C = observer_controller(sysd, L, K; direct)

w = exp10.(LinRange(-2, 2, 200))
dm = diskmargin(C*sysd, 0, w)

println("Input diskmargin:")
display(diskmargin(C*sysd))
Si = input_sensitivity(sysd, C)
Ti = input_comp_sensitivity(sysd, C)
Ms = hinfnorm2(Si)[1]
f1 = gangoffourplot(sysd, C)
sigmaplot!(Si, lab="Si", sp=1, c=3)
sigmaplot!(Ti, lab="Ti", sp=6, c=3)
Li = C*sysd
f2 = nyquistplot(Li, Ms_circles=[2, Ms], ylims=(-2, 2), xlims=(-3,2), lab="Ms = $Ms")
f3 = marginplot(Li)
vline!([27.2], sp=1, lab="Fundamental limitation") # The crossover bandwidth must be higher than this due to unstable pole 27.2 = 2p
f4 = plot(dm, legend=:bottomleft, plot_title="Input diskmargin")
plot(f1, f2, f3, f4)