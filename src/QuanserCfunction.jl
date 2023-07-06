include("QuanserBindings.jl")

const board_type       = "qube_servo2_usb"
const board_identifier = "0@tcpip://localhost:18920?nagle='off'"
board = QuanserBindings.t_card(0)
result = QuanserBindings.hil_open(board_type, board_identifier, board)