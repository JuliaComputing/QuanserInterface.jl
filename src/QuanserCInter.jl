using StaticArrays
include("QuanserBindings.jl")

const board_type       = "qube_servo2_usb"
const board_identifier = "0@tcpip://localhost:18920?nagle='off'"
board = QuanserBindings.t_card(0)



#result = QuanserBindings.hil_open(board_type, board_identifier, board)

function check(result)
    res = pyconvert(Union{Int, Nothing}, result)
    success = res === nothing
    success || @error("Failed: $result")
    success
end

abstract type QuanserCBackend end

struct CBackend <: QuanserBackend
    card
    digital_channel_write::Vector{UInt32}
    encoder_channel::Vector{UInt32}
    analog_channel_write::Vector{UInt32}
    analog_channel_read::Vector{UInt32}
    encoder_read_buffer::Vector{Int32}
    analog_read_buffer::Vector{Int32}
end

function measure(p::CBackend)
    if !isempty(length(p.encoder_channel))
        result = p.card.read_encoder(p.encoder_channel, length(p.encoder_channel), p.encoder_read_buffer)
        check(result)
    end
    if !isempty(p.analog_channel_read)
        result = p.card.read_analog(p.analog_channel_read, length(p.analog_channel_read), p.analog_read_buffer)
        check(result)
    end
    [p.encoder_read_buffer; p.analog_read_buffer]
end

function control(p::CBackend, u::Vector{Float64})
    result = p.card.write_analog(p.analog_channel_write, length(p.analog_channel_write), u)
    check(result)
    u
end

function initialize(p::CBackend)
    enable = [UInt32(1)]
    result = p.card.write_digital(p.digital_channel_write, 1, enable)
    check(result)
end

finalize(p::PythonBackend) = p.card.close()

