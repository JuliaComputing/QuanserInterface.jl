using HardwareAbstractions
using QuanserInterface

"""
    chirp_experiment(p; f0 = 0.08, f1 = 10, Tf = 28, gain = 0.05, u_max = 10.0)

Perform a chirp experiement on process `p`, chirping logarithmically from frequency `f0` to `f1` during `Tf` seconds. 

Internally, `p` is under feedback control with a P controller of gain `gain`, to which the reference is being chirped.

Returns a matrix of log data with row indices `ti = 1; yi = 2; ui = 3; ri = 4; ei = 5;`

# Arguments:
- `p`: Process
- `f0`: Start frequency
- `f1`: End frequency
- `Tf`: Duration (final time)
- `gain`: P controller gain
- `u_max`: Maximum allowed control signal
"""
function chirp_experiment(p; 
    f0 = 0.08,
    f1 = 10,
    Tf = 28,
    gain = 0.05,
    u_max = 10.0,
)
    initialize(p)
    Ts = p.Ts
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

    reduce(hcat, data)
end


p = QubeServo()

# show_measurements(p) do data
#     display(plot(reduce(hcat, data)'))
# end

D = chirp_experiment(p; Tf = 10)

ti = 1; yi = 2; ui = 3; ri = 4; ei = 5;
tvec = D[ti, :]
fig = plot(tvec, D[yi, :], layout=2, lab="y")
plot!(tvec, D[ri, :], lab="r", sp=1)
plot!(tvec, D[ei, :], lab="e", sp=1)
plot!(tvec, D[ui, :], lab="u", sp=2)


# ==============================================================================
## Estimate model
# ==============================================================================
using ControlSystemIdentification, ControlSystemsBase, Plots
d = iddata(D[2,:], D[3, :], p.Ts)
model = arx(d, 2, 2)
velmodel = tf([1, -1], [p.Ts, 0], p.Ts)*model
bodeplot([model, velmodel])