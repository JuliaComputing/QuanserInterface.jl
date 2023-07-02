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

show_measurements(p) do data
    display(plot(reduce(hcat, data)'))
end

D, fig = chirp_demo(p; Tf = 10)
fig

# ==============================================================================
## Pendulum
# ==============================================================================
using ControlSystemsBase, QuanserInterface, RobustAndOptimalControl
measure = QuanserInterface.measure
normalize_angles(x::Number) = mod(x+pi, 2pi)-pi
normalize_angles(x::AbstractVector) = SA[normalize_angles(x[1]), normalize_angles(x[2]), x[3], x[4], x[5]]

p = QubeServoPendulum(; Ts = 0.005)

sys = QuanserInterface.linearized_pendulum()
sysaug = add_measurement_disturbance(sys, [-1e-8;;], [0; 1;;])
# sysaug = add_low_frequency_disturbance(sys, 1, ϵ = 1e-8)
sysd = c2d(sysaug, p.Ts)

Q1 = Diagonal([100000, 0.1, 100, 1, 0])
Q2 = 100I(1)


R1b = [
    zeros(2, 5)
    [zeros(3, 2) diagm([1, 1, 0.001])]
]
R1bd = c2d(ss(sysaug.A, R1b, I, 0), p.Ts).B
R1 = Symmetric(1000R1bd*R1bd')
R2 = 0.001I(2)

syse = ExtendedStateSpace(sysd, C1=I, B1=I)

prob = LQGProblem(syse, Q1, Q2, R1, R2)
L = lqr(prob)
K = kalman(prob)

L = SMatrix{size(L)...}(L)
K = SMatrix{size(K)...}(K)

C = observer_controller(sysd, L, K)
gangoffourplot(prob)
nyquistplot(C*sysd, Ms_circles=2, ylims=(-2, 2), xlims=(-3,2))
marginplot(C*sysd)

function observer(sys::AbstractStateSpace{<:Discrete}, K)
    A, B, C, D = ssdata(sys)
    ss(A - K*C, [B K], I, 0, sys.timeevol)
end
obs = observer(sysd, K)
xr = SA[0, 0, 0, 0, 0]

x0 = SA[0, 0.0, 0, 0.0, 0]

function controller(u, y, obsfilter)
    θ = y[2] - pi
    xh = obsfilter([u; y[1]; θ])
    int_th = deg2rad(20)
    obsfilter.state[end] = clamp(obsfilter.state[end], -int_th, int_th)
    xh[end] = clamp(xh[end], -int_th, int_th)
    @show size(xh)
    @show θ, xh[2]
    xe = normalize_angles(xr - xh)
    u = L*xe
    if abs(normalize_angles(θ)) < 0.4 # Use stabilizing controller
        u, xh
    else
        obsfilter.state[end] = 0
        [0.0], xh
    end
end


function balance_demo(p; 
    u_max = 2.0,
    Tf = 10,
    controller = controller,
)

    initialize(p)
    Ts = p.Ts
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float64}}(undef, 0)
    sizehint!(data, N)
    obsfilter = SysFilter(obs, copy(x0))
    t_start = time()
    y_start = measure(p)
    # y_start .* [1, 1]
    try
        GC.gc()
        GC.enable(false)
        u = [0.0]
        for i = 1:N
            @periodically Ts begin 
                t = time() - t_start
                y = measure(p) - y_start # Subtract initial position for a smoother experience
                u, xh = controller(u, y, obsfilter)
                u = @. clamp(u, -u_max, u_max)
                control(p, Vector(u))
                log = [t; y; xh; u]
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
D = balance_demo(p; u_max=8, Tf = 20)

function plotD(D)
    tvec = D[1, :]
    y = D[2:3, :]'
    # y[:, 2] .-= pi
    xh = D[4:8, :]'
    u = D[9, :]
    plot(tvec, xh, layout=6)
    plot!(tvec, y, sp=[1 2])
    plot!(tvec, u, sp=6, lab="u")
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

psim = QubeServoPendulumSimulator3()
psim.x = SA[0, 0.5pi, 0.0, 0]

function nullcontroller(args...)
    u, xh = controller(args...)
    0*u, xh
end

D = balance_demo(psim; u_max=8, Tf = 3, controller = nullcontroller)

using JuliaSimControl
Afu, Bfu = JuliaSimControl.linearize(QuanserInterface.furuta, zeros(4), [0], psim.p, 0)

plotD(D)