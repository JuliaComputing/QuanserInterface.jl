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
# gmffilt = SysFilter(Cgmf)
#

using QuanserInterface: energy
measure = QuanserInterface.measure

function controller(u, y, θ̇, obsfilter)
    θ = y[2] - pi
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
        u = [80*(E - energy(0,0))*sign(θ̇*cos(θ)) - 0.2*y[1]] # Bias slightly towards center
        @. u = clamp(u, -10, 10)
    end
    u, xh, E
end


function swingup_balance_demo(p; 
    u_max = 2.0,
    Tf = 10,
    controller = controller,
    obsfilter = SysFilter(obs, copy(x0)),
)

    Ts = p.Ts
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float64}}(undef, 0)
    sizehint!(data, N)
        
    simulation = processtype(process) isa SimulatedProcess
    
    if simulation
        u0 = 0.0
    else
        u0 = 0.5QuanserInterface.go_home(process, r = 0, K = 0.05, Ki=0.2, Kf=0.02)
        @show u0
    end
    y = QuanserInterface.measure(process)
    if !simulation
        @info "Starting $(simulation ? "simulation" : "experiment") from y: $y, waiting for your input..."
        readline()
    end

    yo = zeros(2)
    dyf = zeros(2)
    t_start = time()
    try
        GC.gc()
        GC.enable(false)
        u = [0.0]
        oob = 0
        for i = 1:N
            @periodically_yielding Ts begin 
                t = time() - t_start
                y = measure(p)
                dy = (y - yo) ./ Ts
                @. dyf = 0.5dyf + 0.5dy

                u, xh, E = controller(u, y, dyf[2], obsfilter)
                u = @. clamp(u, -u_max, u_max)
                if !(-deg2rad(110) <= y[1] <= deg2rad(110))
                    u = [-0.5*y[1]]
                    @warn "Correcting"
                    oob += 20
                    if oob > 600
                        @error "Out of bounds"
                        break
                    end
                else
                    oob = max(0, oob-1)
                end
                @show u
                control(p, Vector(u))
                log = [t; y; xh; u; E]
                push!(data, log)
                yo = y
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

##
# home!(process, 45)
D = swingup_balance_demo(process; u_max=10, Tf = 12)
plotD(D)

##
using DelimitedFiles
cd(@__DIR__)
# writedlm("swingup.csv", [["t" "angle_arm" "angle_pend" "control_input"]; D[[1,2,3,8], :]'], ',')