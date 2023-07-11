#=
This script estimates a simple friction model consisting of Coulomb friction and a low-order polynomial viscous friction model. The experiment assumes that the servo is equipped with the inertia disc, and runs with a constant velocity for a while, and then increases the velocity. 

The transition periods between velocities is filtered out from the estimation data so that we can make the assumption that the acceleration is always zero. This allows us to disregard inertial effects and model only friction behavor.
=#
using DelimitedFiles

# ==============================================================================
## Collect friction-estimation data
# ==============================================================================
using DiscretePIDs
function vel_control(p; 
    Tf = 10,
    r,
    K,
    Ti,
    Td = false,
    umax = 10.0,
)
    initialize(p)
    Ts = p.Ts
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float64}}(undef, 0)
    sizehint!(data, N)
    t_start = time()
    y_start = measure(p)[]

    pid = DiscretePID(; K, Ti, Td, umax, umin=-umax, Ts, b=0.2)

    y_old = 0.0
    ydf = 0.0

    try
        GC.gc()
        GC.enable(false)
        for i = 1:N
            @periodically_yielding Ts begin 
                t = time() - t_start
                y = measure(p)[] - y_start # Subtract initial position for a smoother experience
                yd = (y - y_old) / Ts
                ydf = 0.9*ydf + 0.1*yd
                # r = 45sin(2π*freq*t)
                u = pid(r(t), ydf)
                control(p, [u])
                @show r(t)
                log = [t, y, ydf, u, r(t)]
                push!(data, log)
                y_old = y
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

p = QubeServo()
##
r = t->-(2 + 2floor(t/4)^2) # Velocity reference, negate this to collect for negative velocities
D = vel_control(p; r, Tf = 55, K=0.02, Ti = 0.1)

# writedlm("frictionexperiment_neg.csv", [["t" "y" "yd_filtered" "u" "r"]; D'], ',')

# ==============================================================================
## Estimate friction model
# ==============================================================================
using DelimitedFiles, DSP
Dp = readdlm("frictionexperiment.csv", ',')
Dn = readdlm("frictionexperiment_neg.csv", ',')
D = identity.([Dp[2:end, :]; Dn[2:end, :]])' # Use both positive and negative velocities
tvec = D[1, :]
y = D[2, :]
yd = D[3, :]
ydc = centraldiff(y) ./ p.Ts
u = D[4, :]
r = D[5, :]
plot(tvec, [yd u r], lab=["yd" "u" "r"], layout=(2,1), sp=[1 2 1], ylims=(0, Inf))



## Data selection.
# To fit the friction model, we select data without significant acceleration, this way we avoid having to model the inertial properties. We also only select small velocities that are relevant for the pendulum swingup
ydd = centraldiff(yd) ./ p.Ts
nf = 20
yddf = filtfilt(ones(nf)./nf, ydd)
absacc = abs.(yddf)
plot(tvec, [yddf, absacc])
smallacc = (absacc .< 2) .& (abs.(yd) .!= 0) .& (abs.(yd) .< 35) # Tuned by looking at the plot above

u1 = u[smallacc]
yd1 = yd[smallacc]

# This plot indicates that the breakaway torque (motor command) is around 0.11
scatter(yd1, u1, ylabel="Input", xlabel="Velocity", framestyle=:zerolines)

# Least squares ================================================================
# We fit a third order polynomial + Coulomb friction to the data
signsquare(x) = sign(x) * x^2
sc = 35
scc = 1
yds = yd1 ./ sc # Scaling for numerical stability
A = [scc*sign.(yd1) yds signsquare.(yds) yds.^3]
w = A \ u1

w[1] *= scc
w[2] /= sc
w[3] /= sc^2
w[4] /= sc^3
# Plotting =====================================================================
ydvec = range(-35, 35, length=1001)
uh = [sign.(ydvec) ydvec signsquare.(ydvec) ydvec.^3] * w
plot!(ydvec, uh, lab="Model",  framestyle=:zerolines, title="Friction model")

smoothsign(x) = tanh(5*x)

uh = [smoothsign.(ydvec) ydvec signsquare.(ydvec) ydvec.^3] * w
plot!(ydvec, uh, lab="Model",  framestyle=:zerolines, title="Friction model")
