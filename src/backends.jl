abstract type QuanserBackend end

# ==============================================================================
## Preference handling
# ==============================================================================

"""
    set_board(board)

Set the board to use, the default is `qube_servo3_usb`.
"""
function set_board(board)
    @set_preferences!("board" => board)
    @info("New board set")
end

function get_board()
    @load_preference("board", "qube_servo3_usb")
end

"""
    set_default_backend(backend)

Set the default backend to use, the default is `"c"`, but you can also choose `"python"` if it's installed and PythonCall is loaded manually.
"""
function set_default_backend(backend)
    @set_preferences!("default_backend" => lowercase(backend))
    @info("New default backend set")
end

function get_default_backend()
    @load_preference("default_backend", "c")
end



# ==============================================================================
## C backend
# ==============================================================================

const cardC = Ref(QuanserBindings.t_card(Int('0')))

struct CBackend <: QuanserBackend
    cardC::Any
    digital_channel_write::Vector{UInt32}
    encoder_channel::Vector{UInt32}
    analog_channel_write::Vector{UInt32}
    analog_channel_read::Vector{UInt32}
    encoder_read_buffer::Vector{Int32}
    analog_read_buffer::Vector{Int32}
end

function checkC(result)
    success = result == 0
    res = QuanserInterface.QuanserBindings.tag_error(abs(result))
    success || @error("Failed: $res $result")
    success
end

function measure(p::CBackend)
    if !isempty(length(p.encoder_channel))
        result = QuanserBindings.hil_read_encoder(p.cardC, p.encoder_channel, length(p.encoder_channel), p.encoder_read_buffer)
        checkC(result)
    end
    if !isempty(p.analog_channel_read)
        result = QuanserBindings.hil_read_analog(p.cardC,p.analog_channel_read, length(p.analog_channel_read), p.analog_read_buffer)
        checkC(result)
    end
    [p.encoder_read_buffer; p.analog_read_buffer]
end

function control(p::CBackend, u::Vector{Float64})
    result = QuanserBindings.hil_write_analog(p.cardC,p.analog_channel_write, length(p.analog_channel_write), u)
    checkC(result)
    u
end

function initialize(p::CBackend)
    enable = [UInt32(1)]
    result = QuanserBindings.hil_write_digital(p.cardC,p.digital_channel_write, 1, enable)
    checkC(result)
end

finalize(p::CBackend) = QuanserBindings.hil_close(p.cardC)

# ==============================================================================
## Backend loading
# ==============================================================================


function load_default_backend(; kwargs...) 
    backend = get_default_backend()
    if backend == "python"
        ext = Base.get_extension(@__MODULE__, :QuanserInterfacePythonCallExt)
        if ext === nothing
            error("""Python extension not loaded, manually install and import PythonCall to use the python backend. Otherwise choose the c backend using set_default_backend("c").""")
        else
            pb = ext.PythonBackend
        end
        load_default_backend(pb; kwargs...)
    elseif backend == "c"
        load_default_backend(CBackend; kwargs...)
    else
        error(""" Unknown backend: $(backend), choose "python" or "c" or implement your own $backend backend""")
    end
end

function load_default_backend(::Type{CBackend}; 
    digital_channel_write::Vector{UInt32},
    encoder_channel::Vector{UInt32},
    analog_channel_write::Vector{UInt32},
    analog_channel_read::Vector{UInt32},
    encoder_read_buffer::Vector{Int32},
    analog_read_buffer::Vector{Int32},
    board = get_board(),
    board_identifier = "0",
)

    if QuanserBindings.hil_is_valid(cardC[]) == Int8(1)
        QuanserBindings.hil_close(cardC[])
    end
    QuanserBindings.hil_close_all()
    # board_identifier = "0@tcpip://localhost:18920?nagle='off'"
    try_twice() do
        result = QuanserBindings.hil_open(board, board_identifier, cardC)
        checkC(result)
        if QuanserBindings.hil_is_valid(cardC[]) != Int8(1)
            error("Failed: Invalid tag card")
        end
    end
    b = CBackend(
        cardC[],
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

"""
    try_twice(f)
Ja ja om du trugar
"""
function try_twice(f)
    try
        f()
    catch e
        if occursin("HILError: -1073", string(e))
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
