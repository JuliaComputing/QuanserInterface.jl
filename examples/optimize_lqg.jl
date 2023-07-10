using Ipopt, OptimizationMOI, Optimization
MOI = OptimizationMOI.MOI
using ImplicitDifferentiation

function triangular(x)
    m = length(x)
    n = round(Int, sqrt(2m-1))
    T = zeros(eltype(x), n, n)
    k = 1
    for i = 1:n, j = i:n
        T[i,j] = x[k]
        k += 1
    end
    T
end
invtriangular(T) = [T[i,j] for i = 1:size(T,1) for j = i:size(T,1)]

"A helper function that simulates the closed-loop system"
function sim(S, PS; Tf)
    res1 = step(S, Tf)   # Simulate S
    res2 = step(PS, Tf)  # Simulate PS
    res1, res2
end
# function sim(G)
#     Gd = c2d(G, 0.1, :tustin)   # Discretize the system
#     res1 = step(Gd[1, 1], 0:0.1:15) # Simulate S
#     res2 = step(Gd[1, 2], 0:0.1:15) # Simulate PS
#     res1, res2
# end

function systemslqr(params::AbstractVector, P)
    n2 = length(params) ÷ 2
    Qchol = triangular(params[1:n2])
    Rchol = triangular(params[n2+1:2n2])
    et = eltype(params)
    Q = Qchol'Qchol
    R = Rchol'Rchol
    L = lqr(P, Q, et.(Q2)) # It's important that the last matrix has the correct type
    K = kalman(P, R, et.(R2))
    C = observer_controller(P, L, K)
    S, PS, CS, T = gangoffour(P, C, minimal=false) # [S PS; CS T]
    S = input_sensitivity(P, C)
    C, S, PS, CS, T, Q, R
end
function optimize_lqq(P, Q1, Q2, R1, R2; Tf = 2, CSweight = 0.0001, Msc=5.0)
    Ω = ControlSystemsBase._default_freq_vector(P, Val(:bode))
    

    "The cost function to optimize"
    function cost(params::AbstractVector, P) 
         # Noise amplification penalty
        C, S, PS, CS, T = systemslqr(params, P)
        res1, res2 = sim(S, PS; Tf)
        R, _ = sigma(CS, Ω)
        CSR = sum(R .*= Ω')               # frequency-weighted noise sensitivity
        perf = mean(abs2, res1.y .*= res1.t') + mean(abs2, res2.y .*= res2.t')
        return perf + CSweight * CSR # Blend objectives together
    end

    @views function constraints(res, params::AbstractVector, P)
        C, S, PS, CS, T = systemslqr(params, P)
        # res .= hinfnorm(S, tol=1e-7)[1]
        res .= maximum(sigma(S, Ω)[1])
        nothing
    end
    

    params = [invtriangular(cholesky(Q1).U); invtriangular(cholesky(R1).U)]
    
    fopt = OptimizationFunction(cost, Optimization.AutoFiniteDiff(); cons=constraints)
    prob = OptimizationProblem(fopt, params, P;
        lb    = 0.5*abs.(params),
        ub    = 2*abs.(params),
        ucons = fill(float(Msc), 1),
        lcons = fill(-Inf, 1),
    )
    solver = Ipopt.Optimizer()
    MOI.set(solver, MOI.RawOptimizerAttribute("print_level"), 5)
    MOI.set(solver, MOI.RawOptimizerAttribute("tol"), 1e-3)
    MOI.set(solver, MOI.RawOptimizerAttribute("acceptable_tol"), 1e-1)
    MOI.set(solver, MOI.RawOptimizerAttribute("max_iter"), 1000)
    MOI.set(solver, MOI.RawOptimizerAttribute("max_wall_time"), 20.0)
    MOI.set(solver, MOI.RawOptimizerAttribute("constr_viol_tol"), 1e-2)
    MOI.set(solver, MOI.RawOptimizerAttribute("acceptable_constr_viol_tol"), 0.1)
    MOI.set(solver, MOI.RawOptimizerAttribute("acceptable_iter"), 5)
    MOI.set(solver, MOI.RawOptimizerAttribute("acceptable_obj_change_tol"), 1e-2)
    MOI.set(solver, MOI.RawOptimizerAttribute("hessian_approximation"), "limited-memory")
    res = solve(prob, solver)
    C, S, PS, CS, T, Q, R = systemslqr(res.u, P)
    (; res, C, S, PS, CS, T, Msc, Q, R)
end

res = optimize_lqq(sysd, Q1, Q2, R1 + 1e-8I, R2; Tf = 2, CSweight = 0.0005, Msc=2.5)
P = sysd
(; C, S, PS, CS, T, Msc, Q, R) = res
r1, r2 = sim(S, PS; Tf= 2)
lab = "Optimized"
f1 = plot(
    r1;
    title     = "Time response",
    lab       = lab * " \$r → e\$",
    legend    = :bottomright,
    fillalpha = 0.05,
    linealpha = 0.8,
    c         = 1,
    layout    = (2,1), sp=1,
)
plot!(
    r2;
    lab       = lab * " \$d → e\$",
    fillalpha = 0.05,
    linealpha = 0.8,
    c         = 1,
    sp        = 2
)
f2 = gangoffourplot(P, C, title="", xlabel="", Ms_lines=nothing)
Si = input_sensitivity(P, C)
bodeplot!(f2, Si, sp=1, plotphase=false)
hline!([Msc]; l=:dashdot, c=1, lab="Constraint", ylims=(9e-2, Inf))
f3 = nyquistplot(C*P; Ms_circles=Msc, ylims=(-2.1, 1.1), xlims=(-2.1, 1.2), lab)
plot(f1, f2, f3; layout=(1, 3), size=(1800, 700), bottommargin=2Plots.mm)
#
Q1 = Q
R1 = R