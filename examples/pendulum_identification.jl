#= 
This script performs maximum likelihood estimation of the parameters of the pendulum. The first estimation optimizes over the paramters of the pendulum only, while the second estimation optimizes also the dynamics noise covariance. The call to readdlm can be replaced with whatever data you have available, as long as you can get a recording of the input and two outputs.

The estimation uses an UnscentedKalmanFilter to perform state estimation, the system is unstable and integrates a force disturbance, and performing identification using a state estimator is a good way of handling both those problems.
=#
cd(@__DIR__)
using DelimitedFiles, Plots
N = 250
D = readdlm("swingup.csv", ',')[2:N, :] .|> identity

t = D[:, 1]
y = D[:, 2:3]
u = D[:, 4]


plot(t, y, layout=3, lab=["arm" "pend"])
plot!(t, u, sp=3, lab="u")

# ==============================================================================
## State estimation
# ==============================================================================
using LowLevelParticleFilters
import LowLevelParticleFilters as llpf
using Distributions

yvv = SVector{2}.(eachrow(y))
uvv = SVector{1}.(eachrow(u))

Ts = median(diff(t))
psim = QubeServoPendulumSimulator(; Ts)
ny, nu = 2, 1 
R1 = kron(LowLevelParticleFilters.double_integrator_covariance(Ts, 1), diagm([100, 100])) + 1e-9I
R2 = 2pi/2048 * I(2)
x0 = SA[yvv[1]..., 0, 0.0]
psim.x = x0
R10 = copy(10R1)
R10[1,1] *= 1000000 # Not sure about the initial arm angle
d0 = MvNormal(x0, R10)
kf = UnscentedKalmanFilter(psim.ddyn, psim.measurement, R1, R2, d0; ny, nu, psim.p)
fsol = forward_trajectory(kf, uvv, yvv, llpf.parameters(kf))
solplot = plot(y, layout = 5, sp=[1 2], lab=["arm" "pend"])
plot!(centraldiff(y) ./ Ts, sp=[3 4], lab=["arm ω" "pend ω"])
solplot = plot!(fsol, ploty=false, plotx=false, name="Initial", c=2)

# ==============================================================================
## Optimization
# ==============================================================================

# typical_magnitude = SA[2.0, 3]
# Λ = Diagonal(1 ./ (typical_magnitude.^2))

function cost(plog::Vector{T}) where T
    p = exp10.(plog)
    d0i = MvNormal(T.(d0.μ), d0.Σ)
    kf = UnscentedKalmanFilter(psim.ddyn, psim.measurement, R1, R2, d0i; ny, nu, p)
    try
        # return LowLevelParticleFilters.sse(kf, uvv, yvv, p, Λ)
        return -LowLevelParticleFilters.loglik(kf, uvv, yvv, p)
    catch
        return T(Inf)
    end
end


p0 = log10.([psim.p...])
cost(p0)

using Optim, Optim.LineSearches
res = Optim.optimize(
    cost,
    p0,
    BFGS(linesearch=LineSearches.BackTracking()),
    Optim.Options(
        show_trace = true,
        show_every = 5,
        iterations = 1000,
        time_limit = 130,
    ),
    autodiff = :forward,
)


popt = exp10.(res.minimizer)
llpf.reset!(kf)
fsolopt = forward_trajectory(kf, uvv, yvv, popt)
plot!(solplot, fsolopt, ploty=false, plotx=false, plotu=false, name="Optimized", c=3)

## Compare initial to optimized parameters
bar([p0 res.minimizer], xticks=(1:length(p0), string.(keys(psim.p))), xlabel="Parameter", ylabel="Value log10", lab=["Initial" "Optimized"], alpha=0.5)


# ==============================================================================
## Optimization of covariance matrix restricted to force disturbance
# ==============================================================================

function costRf(plogR::Vector{T}) where T
    pR = exp10.(plogR)
    p = pR[1:np]
    p[end] = 9.81 # Fix g
    Σf = pR[np+1:end]
    R1 = kron(LowLevelParticleFilters.double_integrator_covariance(Ts, 1), diagm(Σf)) + 1e-9I
    d0i = MvNormal(T.(d0.μ), d0.Σ) # Make sure d0 has correct type of autodiff
    kf = UnscentedKalmanFilter(psim.ddyn, psim.measurement, R1, R2, d0i; ny, nu, p)
    try
        # return LowLevelParticleFilters.sse(kf, uvv, yvv, p, Λ)
        return -LowLevelParticleFilters.loglik(kf, uvv, yvv, p) - logpdf(MvNormal(p0Rf, 2), plogR)
    catch
        return T(Inf)
    end
end

np = length(p0)
p0Rf = [p0; log10.([1, 1])]

resRf = Optim.optimize(
    costRf,
    p0Rf,
    BFGS(),
    Optim.Options(
        show_trace = true,
        show_every = 1,
        iterations = 1000,
        time_limit = 30,
    ),
    autodiff = :forward,
)

poptRf = exp10.(resRf.minimizer[1:np])
poptRf[end] = 9.81 # Fix g
Roptf = kron(LowLevelParticleFilters.double_integrator_covariance(Ts, 1), diagm(exp10.(resRf.minimizer[np+1:end]))) + 1e-9I

kfoptRf = UnscentedKalmanFilter(psim.ddyn, psim.measurement, Roptf, R2, d0; ny, nu, p=poptRf)

llpf.reset!(kfoptRf)
fsoloptRf = forward_trajectory(kfoptRf, uvv, yvv, poptRf)
plot!(solplot, fsoloptRf, ploty=false, plotx=false, plotu=false, name="Optimized Rf", c=4)

## Compare initial to optimized parameters
bar([p0 resRf.minimizer[1:np]], xticks=(1:length(p0), string.(keys(psim.p))), xlabel="Parameter", ylabel="Value log10", lab=["Initial" "Optimized Rf"], alpha=0.5)
