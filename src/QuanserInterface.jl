__precompile__(false)
module QuanserInterface

using StaticArrays
using PythonCall
using HardwareAbstractions
import HardwareAbstractions: control, measure, num_inputs, num_outputs, inputrange, outputrange, isstable, isasstable, sampletime, bias, initialize, finalize

function check(result)
    res = pyconvert(Union{Int, Nothing}, result)
    success = res === nothing
    success || @error("Failed: $result")
    success
end

# abstract type QuanserBackend end

# struct PythonBackend <: QuanserBackend
#     card::PyObject
#     digital_channel_write::Vector{UInt32}
#     encoder_channel::Vector{UInt32}
#     analog_channel::Vector{UInt32}
#     encoder_buffer::Vector{Int32}
# end

# function measure(p::PythonBackend, channel)
#     result = card.read_encoder(p.encoder_channel, length(p.encoder_channel), p.angle_buffer)
#     check(result)
#     p.angle_buffer
# end

# function control(p::PythonBackend, u::Vector{Float64})
#     result = p.card.write_analog(p.analog_channel, length(p.analog_channel), u)
#     check(result)
#     u
# end

# function initialize(p::PythonBackend)
#     enable = [UInt32(1)]
#     result = p.card.write_digital(p.digital_channel_write, 1, enable)
#     check(result)
# end

# finalize(p::PythonBackend) = p.card.close()


# function load_default_backend(
#     digital_channel_write::Vector{UInt32},
#     encoder_channel::Vector{UInt32},
#     analog_channel::Vector{UInt32},
#     encoder_buffer::Vector{Int32},
# )
#     pyimport("sys")
#     sys.path.append("/home/fredrikb/quanser")
#     HIL = pyimport("quanser.hardware" => "HIL")
#     PythonBackend(
#         HIL("qube_servo3_usb", "0"),
#         digital_channel_write,
#         encoder_channel,
#         analog_channel,
#         encoder_buffer,
#     )
# end



# # ==============================================================================
# ## QubeServo
# # ==============================================================================

# abstract type AbstractQubeServo <: PhysicalProcess end
# @with_kw struct QubeServo{B <: QuanserBackend} <: AbstractQubeServo
#     Ts::Float64 = 0.01
#     backend::B  = load_default_backend(
#         digital_channel_write = [UInt32(0)],
#         encoder_channel       = [UInt32(0)],
#         analog_channel        = [UInt32(0)],
#         angle_buffer          = zeros(Int32, 1),
#     )
# end


# control(p::AbstractQubeServo, u::Vector{Float64}) = control(p.backend)
# control(p::AbstractQubeServo, u::Number) = control(p, [u])

# function measure(p::QubeServo)
#     y = measure(p.backend)
#     y .* 360 / 2048 # Degrees
# end


# num_inputs(p::AbstractQubeServo)  = 1
# inputrange(p::AbstractQubeServo)  = [(-10,10)]

# num_outputs(p::QubeServo) = 1
# outputrange(p::QubeServo) = [(-10,10)]
# isstable(p::QubeServo)    = true
# isasstable(p::QubeServo)  = true # Friction
# sampletime(p::QubeServo)  = p.Ts
# bias(p::QubeServo)        = 0.0
# initialize(p::AbstractQubeServo) = initialize(p.backend)
# finalize(p::QubeServo) = finalize(p.backend)










sys = pyimport("sys")
sys.path.append("/home/fredrikb/quanser")
HIL = pyimport("quanser.hardware" => "HIL")
card = HIL("qube_servo3_usb", "0")
channels = UInt32[0, 1]
channel_arm = UInt32[0]
channel_pend = UInt32[1]
num_channels = length(channels)
buffer = zeros(Int32, num_channels)

buffer_arm = zeros(Int32, 1)
buffer_pend = zeros(Int32, 1)
card.read_encoder(channels, num_channels, buffer); buffer

data = typeof(buffer)[]
# for i = 1:1000
#     @periodically 0.01 begin 
#         card.read_encoder(channels, num_channels, buffer)
#         push!(data, copy(buffer))
#     end
# end

# Enable the motor
enable = [UInt32(1)]
digital_channel = [UInt32(0)]
result = card.write_digital(digital_channel, 1, enable);
check(result)

analog_channel = [UInt32(0)]

function measure_arm()
    result = card.read_encoder(channel_arm, 1, buffer_arm)
    check(result)
    buffer_arm[] .* 360 / 2048
end

function control(u::Vector{Float64})
    result = card.write_analog(analog_channel, 1, u)
    check(result)
end

##
control([0.0])
f0 = 0.08
f1 = 10
Tf = 28
Ts = 0.006
N = round(Int, Tf/Ts)
freq = 0.5
gain = 0.05
data = Vector{SVector{5, Float64}}(undef, 0)
sizehint!(data, N)
u_max = 10.0
t_start = time()
y_start = measure_arm()
GC.gc()
GC.enable(false)
for i = 1:N
    @periodically Ts begin 
        t = time() - t_start
        y = measure_arm() - y_start # Subtract initial position for a smoother experience
        # r = 45sin(2π*freq*t)
        r = 25*chirp(t, f0, f1, Tf; logspace=true)
        e = r - y
        u = clamp(gain*e, -u_max, u_max)
        control([u])
        log = SA[t, y, u, r, e]
        push!(data, log)
    end
end
GC.enable(true)
GC.gc()
control([0.0])

D = reduce(hcat, data)

ti = 1
yi = 2
ui = 3
ri = 4
ei = 5

tvec = D[ti, :]
plot(tvec, D[yi, :], layout=2, lab="y")
plot!(tvec, D[ri, :], lab="r", sp=1)
plot!(tvec, D[ei, :], lab="e", sp=1)
plot!(tvec, D[ui, :], lab="u", sp=2)
##

card.close()
# writedlm("/tmp/qubeservo.csv", ["t" "y" "r" "e" "u"; D'], ',')


using ControlSystemIdentification, ControlSystemsBase, Statistics, Plots
d = iddata(D[yi, :], D[ui, :], median(diff(tvec)))
d = detrend(d)
plot(d)
model, _ = newpem(d, 2, zeroD=true)
model = arx(d, 2, 2)
model = subspaceid(d, 2)
plot(
    bodeplot(model, hz=true),
    predplot(model, d),
    simplot(model, d)
)
end
