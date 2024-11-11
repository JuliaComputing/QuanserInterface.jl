using JuliaSimControl
using QuanserInterface, HardwareAbstractions
using DiscretePIDs

function run_control(process, controller; th=5, Tf = 20)
    Ts = process.Ts
    yo = 0.0
    dyf = 0.0
    initialize(process)
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float64}}(undef, 0)
    sizehint!(data, N)

    QuanserInterface.go_home(process)

    local u
    try
        GC.enable(false)
        for i = 1:N
            @periodically Ts begin
                t = (i-1)*Ts
                yp = QuanserInterface.measure(process)
                y = yp[1]
                dy = (y - yo) / Ts
                yo = y
                r = deg2rad(rfun(t))
                dyf = 0.8dyf + 0.2dy
                if controller isa Stateful
                    e = y - r
                    u0 = controller([e; dy], 0, t)
                else
                    u0 = controller(r, y, 0)
                end
                u = clamp(u0, -th, th)
                control(process, [u]) 
                push!(data, [r, y, u, yp[2]])
            end
        end
    catch e
        @error "Terminating" e
    finally
        control(process, [0.0])
        GC.enable(true)
    end
    reduce(hcat, data)
end

function σ(x, p, t)
    e, ė = x
    ė + 7e
end

Ts = 0.005
# process = QubeServoPendulum(; Ts)

function rfun(t)
    if t < 7
        40*sign(sin(2pi/7*t)) + 40
    else
        45sin(2pi/5*t) + 20
    end
end

Tf = 15
th = 3

plotargs1 = (lab=["ref" "y pid" "u pid" "yp pid"], layout=(3,1), sp = [1 1 2 3], c=[10 1 1 1])
controller = DiscretePID(; K = 3.0, Ti = 1.0, Td = 0.3, b=1, Ts)
D1 = run_control(process, controller; th, Tf)
plot(D1'; plot_title="Position control", plotargs1...) |> display

controller = SlidingModeController(σ, (s,t) -> -0.4*sign(s) - 0.1*s) |> Stateful
D2 = run_control(process, controller; th, Tf)
plotargs = (; sp = [1 2 3])
labs = ["y" "u" "yp"]
plot!(D2[2:4, :]'; lab=labs.*" smc", c=2, plotargs...) |> display

controller = SlidingModeController(σ, (s,t) -> -0.4*tanh(3s) - 0.1*s) |> Stateful
D3 = run_control(process, controller; th, Tf)
plot!(D3[2:4, :]'; lab=labs.*" γ smc", c=3, plotargs...) |> display

controller = SlidingModeController(σ, (s,t) -> -0.5*abs(s)^0.5*sign(s)) |> Stateful
D4 = run_control(process, controller; th, Tf)
plot!(D4[2:4, :]'; lab=labs.*" power smc", c=4, plotargs...) |> display

controller = SuperTwistingSMC(0.2, σ, Ts, 1.5) |> Stateful
D5 = run_control(process, controller; th, Tf)
plot!(D5[2:4, :]'; lab=labs.*" ST smc", c=5, plotargs...) |> display

# Control action
labs = ["PID", "SMC", "γ-SMC", "power-SMC", "ST-SMC"]
bar(labs, [
std(D1[3,:]) # PID
std(D2[3,:]) # SMC
std(D3[3,:]) # γ-SMC
std(D4[3,:]) # power-SMC
std(D5[3,:]) # ST-SMC
], title="Control action (std)")

# Control total variation
bar(labs, [
sum(abs, diff(D1[3,:])) # PID
sum(abs, diff(D2[3,:])) # SMC
sum(abs, diff(D3[3,:])) # γ-SMC
sum(abs, diff(D4[3,:])) # power-SMC
sum(abs, diff(D5[3,:])) # ST-SMC
], title="Control action (TV)")

# Tracking error
bar(labs, [
std(D1[1,:] - D1[2,:]) # PID
std(D2[1,:] - D2[2,:]) # SMC
std(D3[1,:] - D3[2,:]) # γ-SMC
std(D4[1,:] - D4[2,:]) # power-SMC
std(D5[1,:] - D5[2,:]) # ST-SMC
], title="Tracking error (std)")

# Pendulum motion
bar(labs, [
sum(abs.(D1[1,:] .- D1[2,:])) # PID
sum(abs.(D2[1,:] .- D2[2,:])) # SMC
sum(abs.(D3[1,:] .- D3[2,:])) # γ-SMC
sum(abs.(D4[1,:] .- D4[2,:])) # power-SMC
sum(abs.(D5[1,:] .- D5[2,:])) # ST-SMC
], title="Pendulum motion (TV)")
