using QuanserInterface
using HardwareAbstractions
using ControlSystemsBase
using QuanserInterface: energy
using Test


rr = Ref([0, pi, 0, 0])
nu  = 1 # number of controls
nx  = 4 # number of states
Ts  = 0.01 # sampling time

function plotD(D, th=0.2)
    tvec = D[1, :]
    y = D[2:3, :]'
    # y[:, 2] .-= pi
    # y[:, 2] .*= -1
    xh = D[4:7, :]'
    u = D[8, :]
    plot(tvec, xh, layout=6, lab=["arm" "pend" "arm ω" "pend ω"] .* " estimate", framestyle=:zerolines)
    plot!(tvec, y, sp=[1 2], lab = ["arm" "pend"] .* " meas", framestyle=:zerolines)
    hline!([-pi pi], lab="", sp=2)
    hline!([-pi-th -pi+th pi-th pi+th], lab="", l=(:black, :dash), sp=2)
    plot!(tvec, centraldiff(y) ./ median(diff(tvec)), sp=[3 4], lab="central diff")
    plot!(tvec, u, sp=5, lab = "u", framestyle=:zerolines)
    plot!(diff(D[1,:]), sp=6, lab="Δt"); hline!([process.Ts], sp=6, framestyle=:zerolines, lab="Ts")
end
normalize_angles(x::Number) = mod(x, 2pi)
normalize_angles(x::AbstractVector) = SA[(x[1]), normalize_angles(x[2]), x[3], x[4]]

function swingup(process; Tf = 10, verbose=true, stab=true, umax=5.0)
    # Ts = process.Ts
    Ts = 0.01
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float64}}(undef, 0)
    sizehint!(data, N)

    simulation = processtype(process) isa SimulatedProcess

    if simulation
        u0 = 0.0
    else
        u0 = 0.5QuanserInterface.go_home(process)
        @show u0
    end
    y = QuanserInterface.measure(process)
    if verbose && !simulation
        verbose && @info "Starting $(simulation ? "simulation" : "experiment") from y: $y, waiting for your input..."
        # readline()
    end
    yo = zeros(2)
    dyf = zeros(2)
    L = [-7.410199310542298 -36.40730995983665 -2.0632501290782095 -3.149033572767301] # State-feedback gain

    try
        GC.gc()
        GC.enable(false)
        t_start = time()
        u = [0.0]
        oob = 0
        for i = 1:N
            @periodically Ts simulation begin 
                t = simulation ? (i-1)*Ts : time() - t_start
                y = QuanserInterface.measure(process)
                dy = (y - yo) ./ Ts
                @. dyf = 0.5dyf + 0.5dy
                xh = [y; dyf]
                xhn = [xh[1], normalize_angles(xh[2]), xh[3], xh[4]]
                r = rr[]
                if !(-deg2rad(110) <= y[1] <= deg2rad(110))
                    u = [-0.5*y[1]]
                    verbose && @warn "Correcting"
                    control(process, u)
                    oob += 20
                    if oob > 600
                        verbose && @error "Out of bounds"
                        break
                    end
                else
                    oob = max(0, oob-1)
                    if stab && abs(normalize_angles(y[2]) - pi) < 0.50
                        verbose && @info "stabilizing"
                        u = clamp.(L*(r - xhn), -10, 10)
                    else
                        # xhn = (process.x) # Try with correct state if in simulation
                        α = y[2] - pi
                        αr = r[2] - pi
                        α̇ = xh[4]
                        E = energy(α, α̇)
                        u = [clamp(80*(E - energy(αr,0))*sign(α̇*cos(α)) - 0.2*y[1], -umax, umax)]
                    end
                    control(process, u)
                end
                verbose && @info "t = $(round(t, digits=3)), u = $(u[]), xh = $xh"
                log = [t; y; xh; u]
                push!(data, log)
                yo = y
            end
        end
    catch e
        @error "Terminating" e
        # rethrow()
    finally
        control(process, [0.0])
        GC.enable(true)
        GC.gc()
    end

    D = reduce(hcat, data)
end

process = QuanserInterface.QubeServoPendulumSimulator(; Ts, p = QuanserInterface.pendulum_parameters(true))
function runplot(process; verbose = isinteractive(), kwargs...)
    rr[][1] = deg2rad(0)
    rr[][2] = pi
    y = QuanserInterface.measure(process)
    if abs(y[2]) > 0.8 || !(-2.5 < y[1] < 2.5)
        @info "Auto homing"
        autohome!(process)
    end
    global D
    D = swingup(process; verbose, kwargs...)
    isinteractive() && plotD(D)
end

runplot(process; Tf = 15)

@test norm(D[2:3, end] .- [0, pi]) < 1e-6