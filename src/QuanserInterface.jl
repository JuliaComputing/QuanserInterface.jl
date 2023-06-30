__precompile__(false)
module QuanserInterface

export QubeServo


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

abstract type QuanserBackend end

struct PythonBackend <: QuanserBackend
    card::Py
    digital_channel_write::Vector{UInt32}
    encoder_channel::Vector{UInt32}
    analog_channel::Vector{UInt32}
    encoder_buffer::Vector{Int32}
end

function measure(p::PythonBackend)
    result = p.card.read_encoder(p.encoder_channel, length(p.encoder_channel), p.encoder_buffer)
    check(result)
    p.encoder_buffer
end

function control(p::PythonBackend, u::Vector{Float64})
    result = p.card.write_analog(p.analog_channel, length(p.analog_channel), u)
    check(result)
    u
end

function initialize(p::PythonBackend)
    enable = [UInt32(1)]
    result = p.card.write_digital(p.digital_channel_write, 1, enable)
    check(result)
end

finalize(p::PythonBackend) = p.card.close()


function load_default_backend(; 
    digital_channel_write::Vector{UInt32},
    encoder_channel::Vector{UInt32},
    analog_channel::Vector{UInt32},
    encoder_buffer::Vector{Int32},
)
    sys = pyimport("sys")
    sys.path.append("/home/fredrikb/quanser")
    HIL = pyimport("quanser.hardware" => "HIL")
    PythonBackend(
        try_twice(()->HIL("qube_servo3_usb", "0")),
        digital_channel_write,
        encoder_channel,
        analog_channel,
        encoder_buffer,
    )
end

"""
    try_twice(f)
Ja ja om du trugar
"""
function try_twice(f)
    try
        f()
    catch e
        if occursin("HILError: -1073", e.msg)
            sleep(0.2)
            return f()
        else
            rethrow()
        end
    end
end


# ==============================================================================
## QubeServo
# ==============================================================================

abstract type AbstractQubeServo <: PhysicalProcess end
@kwdef struct QubeServo{B <: QuanserBackend} <: AbstractQubeServo
    Ts::Float64 = 0.01
    backend::B  = load_default_backend(
        digital_channel_write = [UInt32(0)],
        encoder_channel       = [UInt32(0)],
        analog_channel        = [UInt32(0)],
        encoder_buffer        = zeros(Int32, 1),
    )
end


control(p::AbstractQubeServo, u::Vector{Float64}) = control(p.backend, u)
control(p::AbstractQubeServo, u::Number) = control(p, [u])

function measure(p::QubeServo)
    y = measure(p.backend)
    y .* 360 / 2048 # Degrees
end


num_inputs(p::AbstractQubeServo)  = 1
inputrange(p::AbstractQubeServo)  = [(-10,10)]

num_outputs(p::QubeServo) = 1
outputrange(p::QubeServo) = [(-10,10)]
isstable(p::QubeServo)    = true
isasstable(p::QubeServo)  = true # Friction
sampletime(p::QubeServo)  = p.Ts
bias(p::QubeServo)        = 0.0
initialize(p::AbstractQubeServo) = initialize(p.backend)
finalize(p::QubeServo) = finalize(p.backend)

##


##


end
