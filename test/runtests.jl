using HardwareAbstractions
using QuanserInterface
using Test
cd(@__DIR__)


function plotD(D)
    tvec = D[1, :]
    y = D[2:3, :]'
    y[:, 2] .-= pi
    xh = D[4:7, :]'
    u = D[8, :]
    E = D[9, :]
    plot(tvec, xh, layout=6)
    plot!(tvec, y, sp=[1 2])
    plot!(tvec, u, sp=5, lab="u")
    plot!(tvec, E, sp=6, lab="E")
end

# ==============================================================================
## Pendulum
Ts = 0.01
process = QubeServoPendulum(; Ts)
psim = QubeServoPendulumSimulator(; Ts)
# home!(process, 43)
##
# ==============================================================================
include("../examples/furuta_lqg.jl")

##
velfilt1 = c2d(ss(tf([50, 0], [1, 50])), Ts, :tustin) |> SysFilter
velfilt2 = c2d(ss(tf([50, 0], [1, 50])), Ts, :tustin) |> SysFilter
gmffilt = SysFilter(Cgmf)
#

using QuanserInterface: energy
measure = QuanserInterface.measure

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
        rethrow()
    finally
        control(p, [0.0])
        GC.enable(true)
        GC.gc()
    end

    D = reduce(hcat, data)
end

##

D = balance_demo(process; u_max=8, Tf = 20)
plotD(D)
