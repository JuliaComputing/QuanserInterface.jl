module QuanserInterfacePythonCallExt
using PythonCall
using Preferences
import QuanserInterface: measure, control, initialize, finalize, QuanserBackend, load_default_backend, get_board, try_twice
export set_quanser_python_path, get_quanser_python_path, PythonBackend

"""
    set_quanser_python_path(path)

Set the path to the Quanser python HIL interface, the default if none is set is `~/quanser`.
"""
function set_quanser_python_path(path)
    @set_preferences!("python_path" => path)
    @info("New python path set")
end

function get_quanser_python_path()
    @load_preference("python_path", "~/quanser")
end


# ==============================================================================
## Python backend
# ==============================================================================

const HIL = Ref(Py(nothing))
const card = Ref(Py(nothing))

function check(result)
    res = pyconvert(Union{Int, Nothing}, result)
    success = res === nothing
    success || @error("Failed: $result")
    success
end

struct PythonBackend <: QuanserBackend
    card::Py
    digital_channel_write::Vector{UInt32}
    encoder_channel::Vector{UInt32}
    analog_channel_write::Vector{UInt32}
    analog_channel_read::Vector{UInt32}
    encoder_read_buffer::Vector{Int32}
    analog_read_buffer::Vector{Int32}
end

function measure(p::PythonBackend)
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

function control(p::PythonBackend, u::Vector{Float64})
    result = p.card.write_analog(p.analog_channel_write, length(p.analog_channel_write), u)
    check(result)
    u
end

function initialize(p::PythonBackend)
    enable = [UInt32(1)]
    result = p.card.write_digital(p.digital_channel_write, 1, enable)
    check(result)
end

finalize(p::PythonBackend) = p.card.close()

function load_default_backend(::Type{PythonBackend}; 
    digital_channel_write::Vector{UInt32},
    encoder_channel::Vector{UInt32},
    analog_channel_write::Vector{UInt32},
    analog_channel_read::Vector{UInt32},
    encoder_read_buffer::Vector{Int32},
    analog_read_buffer::Vector{Int32},
    board = get_board(),
    board_identifier = "0",
)

    if HIL[] === Py(nothing)
        @info "Loading quanser Python module"
        sys = pyimport("sys")

        sys.path.append(get_quanser_python_path())
        HIL[] = pyimport("quanser.hardware" => "HIL")
    end
    if card[] !== Py(nothing)
        # If already loaded, close first
        @info "Closing previous HIL card"
        card[].close()
        sleep(0.1)
    end
    @info "Loading HIL card"
    
    card[] = try_twice(()->HIL[](board, board_identifier))
    b = PythonBackend(
        card[],
        digital_channel_write,
        encoder_channel,
        analog_channel_write,
        analog_channel_read,
        encoder_read_buffer,
        analog_read_buffer,
    )

    initialize(b)
    b
end

end