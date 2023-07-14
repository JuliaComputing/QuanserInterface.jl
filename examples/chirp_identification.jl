#=
Perform a system-identification experiement on the DC servo (the Qube Servo with the inertia-disc attachement instead of the pendulum).
=#
using HardwareAbstractions, QuanserInterface, StaticArrays, Plots

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
    amplitude = 45,
    feedback = true,
)
    initialize(p)
    Ts = p.Ts
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float32}}(undef, 0)
    sizehint!(data, N)
    t_start = time()
    y_start = measure(p)
    @info "Starting chirp experiment from f0 = $f0 to f1 = $f1 during $Tf seconds. Feedback: $feedback"
    try
        GC.gc()
        GC.enable(false)
        for i = 1:N
            @periodically Ts begin 
                t = time() - t_start
                y = measure(p) - y_start # Subtract initial position for a smoother experience
                r = amplitude*chirp(t, f0, f1, Tf; logspace=true)
                if feedback
                    e = r - y[1]
                    u = clamp(gain*e, -u_max, u_max)
                else
                    e = 0.0
                    u = clamp(gain*r, -u_max, u_max)
                end
                control(p, [u])
                log = [t; y; u; r; e]
                push!(data, log)
            end
        end
    catch e
        @error "Terminating" e
    finally
        control(p, [0.0])
        GC.enable(true)
        GC.gc()
    end

    reduce(hcat, data)
end



# show_measurements(p) do data
#     display(plot(reduce(hcat, data)'))
# end

if true # Use DC-servo only
    process = QubeServo()
    D = chirp_experiment(process; Tf = 20, f1=8, feedback=true)
else
    process = QubeServoPendulum()
    autohome!(process)
    QuanserInterface.go_home(process)
    D = chirp_experiment(process; Tf = 30, f1=3, feedback=true, amplitude = deg2rad(60), gain=1)
end

##
ny = noutputs(process)
ti = 1; yi = (1:ny) .+ 1; ui = yi[end]+1; ri = ui+1; ei = ri+1;
tvec = D[ti, :]
fig = plot(tvec, D[yi, :]', layout=ny+1, lab="y")
plot!(tvec, D[ri, :], lab="r", sp=1, alpha=0.5)
plot!(tvec, D[ei, :], lab="e", sp=1, alpha=0.5)
plot!(tvec, D[ui, :], lab="u", sp=ny+1)
display(current())

# ==============================================================================
## Save data
# ==============================================================================
using DelimitedFiles
cd(@__DIR__)
writedlm("data/chirp_experiment.csv", [["t" ]], ',')

# ==============================================================================
## Estimate model
# ==============================================================================
using ControlSystemIdentification, ControlSystemsBase, Plots
using ControlSystemsBase: numeric_type
d = iddata(D[2,:], D[3, :], process.Ts)
ControlSystemIdentification.find_nanb(d, 4, 4)
model = arx(d, 2, 1, stochastic=false)
w = exp10.(LinRange(0, 2, 200))
f1 = coherenceplot(d)
f2 = bodeplot(model, w)

x0 = numeric_type(model).([0.0, 0.0])
f3 = simplot(model, d, x0)
f4 = predplot(model, d)
plot(f1,f2,f3,f4)