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
                r = 25*chirp(t, f0, f1, Tf; logspace=true)
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

    ti = 1
    yi = 2
    ui = 3
    ri = 4
    ei = 5

    tvec = D[ti, :]
    fig = plot(tvec, D[yi, :], layout=2, lab="y")
    plot!(tvec, D[ri, :], lab="r", sp=1)
    plot!(tvec, D[ei, :], lab="e", sp=1)
    plot!(tvec, D[ui, :], lab="u", sp=2)
    finalize(p)
    (; D, fig)
end


p = QubeServo()
D, fig = chirp_demo(p; Tf = 10)
fig











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