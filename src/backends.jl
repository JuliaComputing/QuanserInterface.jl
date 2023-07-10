# ==============================================================================
## Preference handling
# ==============================================================================

"""
    set_quanser_python_path(path)

Set the path to the Quanser python HIL interface, the default if none is set is `~/quanser`.
"""
function set_quanser_python_path(path)
    @set_preferences!("python_path" => path)
    @info("New python path set; restart your Julia session for this change to take effect!")
end

function get_quanser_python_path()
    @load_preference("python_path", "~/quanser")
end

"""
    set_board(board)

Set the board to use, the default is `qube_servo3_usb`.
"""
function set_board(board)
    @set_preferences!("board" => board)
    @info("New board set; restart your Julia session for this change to take effect!")
end

function get_board()
    @load_preference("board", "qube_servo3_usb")
end

"""
    set_default_backend(backend)

Set the default backend to use, the default is `"python"`, but you can also choose `"c"`.
"""
function set_default_backend(backend)
    @set_preferences!("default_backend" => lowercase(backend))
    @info("New default backend set; restart your Julia session for this change to take effect!")
end

function get_default_backend()
    @load_preference("default_backend", "python")
end

# ==============================================================================
## Backend loading
# ==============================================================================

abstract type QuanserBackend end

function load_default_backend(; 
    digital_channel_write::Vector{UInt32},
    encoder_channel::Vector{UInt32},
    analog_channel_write::Vector{UInt32},
    analog_channel_read::Vector{UInt32},
    encoder_read_buffer::Vector{Int32},
    analog_read_buffer::Vector{Int32},
)
    backend = get_default_backend()
    if backend == "python"
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
        board = get_board()
        card[] = try_twice(()->HIL[]("qube_servo3_usb", "0"))
        PythonBackend(
            card[],
            digital_channel_write,
            encoder_channel,
            analog_channel_write,
            analog_channel_read,
            encoder_read_buffer,
            analog_read_buffer,
        )
    elseif backend == "c"
        error("c backend not yet implemented")
    else
        error(""" Unknown backend: $(backend), choose "python" or "c" or implement your own $backend backend""")
    end
end

"""
    try_twice(f)
Ja ja om du trugar
"""
function try_twice(f)
    try
        f()
    catch e
        if e isa PyException occursin("HILError: -1073", string(e))
            sleep(0.2)
            try
                return f()
            catch
                @error "Failed twice, is the device power off, or did you forget to call finalize?"
                rethrow()
            end
        else
            rethrow()
        end
    end
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


