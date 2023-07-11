module QuanserBindings

using CEnum

@static if Base.Sys.islinux()
    const hil_sdk = "/opt/quanser/lib/libhil.so"
elseif Base.Sys.isapple()
    const hil_sdk = "/opt/quanser/lib/libhil.dylib"
else
    error("Unsupported platform")
end

const __darwin_off_t = Int64

const __darwin_pid_t = Int32

const __darwin_sigset_t = UInt32

struct __darwin_pthread_handler_rec
    __routine::Ptr{Cvoid}
    __arg::Ptr{Cvoid}
    __next::Ptr{__darwin_pthread_handler_rec}
end

struct _opaque_pthread_attr_t
    __sig::Clong
    __opaque::NTuple{56, Cchar}
end

struct _opaque_pthread_mutex_t
    __sig::Clong
    __opaque::NTuple{56, Cchar}
end

struct _opaque_pthread_mutexattr_t
    __sig::Clong
    __opaque::NTuple{8, Cchar}
end

struct _opaque_pthread_once_t
    __sig::Clong
    __opaque::NTuple{8, Cchar}
end

struct _opaque_pthread_t
    __sig::Clong
    __cleanup_stack::Ptr{__darwin_pthread_handler_rec}
    __opaque::NTuple{8176, Cchar}
end

const __darwin_pthread_attr_t = _opaque_pthread_attr_t

const __darwin_pthread_key_t = Culong

const __darwin_pthread_mutex_t = _opaque_pthread_mutex_t

const __darwin_pthread_mutexattr_t = _opaque_pthread_mutexattr_t

const __darwin_pthread_once_t = _opaque_pthread_once_t

const __darwin_pthread_t = Ptr{_opaque_pthread_t}

const off_t = __darwin_off_t

const pid_t = __darwin_pid_t

const sigset_t = __darwin_sigset_t

const pthread_attr_t = __darwin_pthread_attr_t

const pthread_mutex_t = __darwin_pthread_mutex_t

const pthread_mutexattr_t = __darwin_pthread_mutexattr_t

const pthread_once_t = __darwin_pthread_once_t

const pthread_t = __darwin_pthread_t

const pthread_key_t = __darwin_pthread_key_t

struct __sigaction_u
    data::NTuple{8, UInt8}
end

function Base.getproperty(x::Ptr{__sigaction_u}, f::Symbol)
    f === :__sa_handler && return Ptr{Ptr{Cvoid}}(x + 0)
    f === :__sa_sigaction && return Ptr{Ptr{Cvoid}}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__sigaction_u, f::Symbol)
    r = Ref{__sigaction_u}(x)
    ptr = Base.unsafe_convert(Ptr{__sigaction_u}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__sigaction_u}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct sigaction
    data::NTuple{16, UInt8}
end

function Base.getproperty(x::Ptr{sigaction}, f::Symbol)
    f === :__sigaction_u && return Ptr{__sigaction_u}(x + 0)
    f === :sa_mask && return Ptr{sigset_t}(x + 8)
    f === :sa_flags && return Ptr{Cint}(x + 12)
    return getfield(x, f)
end

function Base.getproperty(x::sigaction, f::Symbol)
    r = Ref{sigaction}(x)
    ptr = Base.unsafe_convert(Ptr{sigaction}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{sigaction}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

function sigaction(arg1, arg2, arg3)
    ccall((:sigaction, hil_sdk), Cint, (Cint, Ptr{sigaction}, Ptr{sigaction}), arg1, arg2, arg3)
end

mutable struct tag_card end

const t_card = Ptr{tag_card}

const t_int = Cint

const t_error = t_int

function hil_open(card_type, card_identifier, card)
    ccall((:hil_open, hil_sdk), t_error, (Ptr{Cchar}, Ptr{Cchar}, Ptr{t_card}), card_type, card_identifier, card)
end

function hil_set_card_specific_options(card, options, options_size)
    ccall((:hil_set_card_specific_options, hil_sdk), t_error, (t_card, Ptr{Cchar}, Csize_t), card, options, options_size)
end

@cenum tag_hil_string_property::UInt32 begin
    PROPERTY_STRING_MANUFACTURER = 0
    PROPERTY_STRING_PRODUCT_NAME = 1
    PROPERTY_STRING_MODEL_NAME = 2
    PROPERTY_STRING_SERIAL_NUMBER = 3
    PROPERTY_STRING_FIRMWARE_VERSION = 4
    PROPERTY_STRING_PRODUCT_SPECIFIC = 128
end

const t_hil_string_property = tag_hil_string_property

function hil_get_string_property(card, property_code, buffer, buffer_size)
    ccall((:hil_get_string_property, hil_sdk), t_error, (t_card, t_hil_string_property, Ptr{Cchar}, Csize_t), card, property_code, buffer, buffer_size)
end

function hil_set_string_property(card, property_code, buffer, buffer_size)
    ccall((:hil_set_string_property, hil_sdk), t_error, (t_card, t_hil_string_property, Ptr{Cchar}, Csize_t), card, property_code, buffer, buffer_size)
end

mutable struct tag_task end

const t_task = Ptr{tag_task}

mutable struct tag_monitor end

const t_monitor = Ptr{tag_monitor}

@cenum tag_clock::Int32 begin
    SYSTEM_CLOCK_4 = -4
    SYSTEM_CLOCK_3 = -3
    SYSTEM_CLOCK_2 = -2
    SYSTEM_CLOCK_1 = -1
    HARDWARE_CLOCK_0 = 0
    HARDWARE_CLOCK_1 = 1
    HARDWARE_CLOCK_2 = 2
    HARDWARE_CLOCK_3 = 3
    HARDWARE_CLOCK_4 = 4
    HARDWARE_CLOCK_5 = 5
    HARDWARE_CLOCK_6 = 6
    HARDWARE_CLOCK_7 = 7
    HARDWARE_CLOCK_8 = 8
    HARDWARE_CLOCK_9 = 9
    HARDWARE_CLOCK_10 = 10
    HARDWARE_CLOCK_11 = 11
    HARDWARE_CLOCK_12 = 12
    HARDWARE_CLOCK_13 = 13
    HARDWARE_CLOCK_14 = 14
    HARDWARE_CLOCK_15 = 15
    HARDWARE_CLOCK_16 = 16
    HARDWARE_CLOCK_17 = 17
    HARDWARE_CLOCK_18 = 18
    HARDWARE_CLOCK_19 = 19
end

const t_clock = tag_clock

@cenum tag_clock_mode::UInt32 begin
    CLOCK_TIMEBASE_MODE = 0
    CLOCK_PWM_MODE = 1
    CLOCK_ENCODER_MODE = 2
    NUMBER_OF_CLOCK_MODES = 3
end

const t_clock_mode = tag_clock_mode

@cenum tag_analog_input_configuration::UInt32 begin
    ANALOG_INPUT_RSE_CONFIGURATION = 0
    ANALOG_INPUT_NRSE_CONFIGURATION = 1
    ANALOG_INPUT_DIFF_CONFIGURATION = 2
    ANALOG_INPUT_PDIFF_CONFIGURATION = 3
    NUMBER_OF_ANALOG_INPUT_CONFIGURATIONS = 4
end

const t_analog_input_configuration = tag_analog_input_configuration

@cenum tag_pwm_mode::UInt32 begin
    PWM_DUTY_CYCLE_MODE = 0
    PWM_FREQUENCY_MODE = 1
    PWM_PERIOD_MODE = 2
    PWM_ONE_SHOT_MODE = 3
    PWM_TIME_MODE = 4
    PWM_ENCODER_EMULATION_MODE = 5
    PWM_RAW_MODE = 6
    NUMBER_OF_PWM_MODES = 7
end

const t_pwm_mode = tag_pwm_mode

@cenum tag_pwm_configuration::UInt32 begin
    PWM_UNIPOLAR_CONFIGURATION = 0
    PWM_BIPOLAR_CONFIGURATION = 1
    PWM_PAIRED_CONFIGURATION = 2
    PWM_COMPLEMENTARY_CONFIGURATION = 3
    NUMBER_OF_PWM_CONFIGURATIONS = 4
end

const t_pwm_configuration = tag_pwm_configuration

@cenum tag_pwm_alignment::UInt32 begin
    PWM_LEADING_EDGE_ALIGNED = 0
    PWM_TRAILING_EDGE_ALIGNED = 1
    PWM_CENTER_ALIGNED = 2
    NUMBER_OF_PWM_ALIGNMENTS = 3
end

const t_pwm_alignment = tag_pwm_alignment

@cenum tag_pwm_polarity::UInt32 begin
    PWM_ACTIVE_LOW_POLARITY = 0
    PWM_ACTIVE_HIGH_POLARITY = 1
    NUMBER_OF_PWM_POLARITIES = 2
end

const t_pwm_polarity = tag_pwm_polarity

@cenum tag_encoder_quadrature_mode::UInt32 begin
    ENCODER_QUADRATURE_NONE = 0
    ENCODER_QUADRATURE_1X = 1
    ENCODER_QUADRATURE_2X = 2
    ENCODER_QUADRATURE_4X = 4
end

const t_encoder_quadrature_mode = tag_encoder_quadrature_mode

@cenum tag_digital_configuration::UInt32 begin
    DIGITAL_OPEN_COLLECTOR_CONFIGURATION = 0
    DIGITAL_TOTEM_POLE_CONFIGURATION = 1
    NUMBER_OF_DIGITAL_CONFIGURATIONS = 2
end

const t_digital_configuration = tag_digital_configuration

@cenum tag_digital_state::UInt32 begin
    DIGITAL_STATE_LOW = 0
    DIGITAL_STATE_HIGH = 1
    DIGITAL_STATE_TRISTATE = 2
    DIGITAL_STATE_NO_CHANGE = 3
    NUM_DIGITAL_STATES = 4
end

const t_digital_state = tag_digital_state

@cenum tag_digital_mode::UInt32 begin
    DIGITAL_MODE_DIN = 0
    DIGITAL_MODE_DOUT = 1
    DIGITAL_MODE_PWM = 2
    DIGITAL_MODE_ENC = 3
    DIGITAL_MODE_SPI = 4
    DIGITAL_MODE_I2C = 5
    DIGITAL_MODE_UART = 6
    NUM_DIGITAL_MODES = 7
end

const t_digital_mode = tag_digital_mode

@cenum tag_hil_integer_property::UInt32 begin
    PROPERTY_INTEGER_VENDOR_ID = 0
    PROPERTY_INTEGER_PRODUCT_ID = 1
    PROPERTY_INTEGER_SUBVENDOR_ID = 2
    PROPERTY_INTEGER_SUBPRODUCT_ID = 3
    PROPERTY_INTEGER_MAJOR_VERSION = 4
    PROPERTY_INTEGER_MINOR_VERSION = 5
    PROPERTY_INTEGER_BUILD = 6
    PROPERTY_INTEGER_REVISION = 7
    PROPERTY_INTEGER_DATE = 8
    PROPERTY_INTEGER_TIME = 9
    PROPERTY_INTEGER_FIRMWARE_MAJOR_VERSION = 10
    PROPERTY_INTEGER_FIRMWARE_MINOR_VERSION = 11
    PROPERTY_INTEGER_FIRMWARE_BUILD = 12
    PROPERTY_INTEGER_FIRMWARE_REVISION = 13
    PROPERTY_INTEGER_FIRMWARE_DATE = 14
    PROPERTY_INTEGER_FIRMWARE_TIME = 15
    PROPERTY_INTEGER_NUMBER_OF_ANALOG_INPUTS = 16
    PROPERTY_INTEGER_NUMBER_OF_ENCODER_INPUTS = 17
    PROPERTY_INTEGER_NUMBER_OF_DIGITAL_INPUTS = 18
    PROPERTY_INTEGER_NUMBER_OF_OTHER_INPUTS = 19
    PROPERTY_INTEGER_NUMBER_OF_ANALOG_OUTPUTS = 20
    PROPERTY_INTEGER_NUMBER_OF_PWM_OUTPUTS = 21
    PROPERTY_INTEGER_NUMBER_OF_DIGITAL_OUTPUTS = 22
    PROPERTY_INTEGER_NUMBER_OF_OTHER_OUTPUTS = 23
    PROPERTY_INTEGER_NUMBER_OF_CLOCKS = 24
    PROPERTY_INTEGER_NUMBER_OF_INTERRUPTS = 25
    PROPERTY_INTEGER_PRODUCT_SPECIFIC = 128
    PROPERTY_INTEGER_HIQ_GYRO_RANGE = 128
    PROPERTY_INTEGER_HIQ_MAGNETOMETER_MODE = 129
    PROPERTY_INTEGER_HIQ_BLDC_MAX_PWM_TICKS = 130
    PROPERTY_INTEGER_HIQ_BLDC_RAMPUP_DELAY = 131
    PROPERTY_INTEGER_HIQ_BLDC_MAX_DUTY_CYCLE = 132
    PROPERTY_INTEGER_HIQ_BLDC_MIN_DUTY_CYCLE = 133
    PROPERTY_INTEGER_HIQ_BLDC_POLE_PAIRS = 134
    PROPERTY_INTEGER_HIQ_PWM_MODE = 135
    PROPERTY_INTEGER_QBUS_NUM_MODULES = 128
    PROPERTY_INTEGER_KOBUKI_UDID0 = 128
    PROPERTY_INTEGER_KOBUKI_UDID1 = 129
    PROPERTY_INTEGER_KOBUKI_UDID2 = 130
end

const t_hil_integer_property = tag_hil_integer_property

@cenum tag_hil_double_property::UInt32 begin
    PROPERTY_DOUBLE_PRODUCT_SPECIFIC = 128
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_0 = 128
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_1 = 129
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_2 = 130
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_3 = 131
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_4 = 132
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_5 = 133
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_6 = 134
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_7 = 135
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_8 = 136
    PROPERTY_DOUBLE_HIQ_SERVO_FINAL_VALUE_CH_9 = 137
    PROPERTY_DOUBLE_KOBUKI_P_GAIN = 128
    PROPERTY_DOUBLE_KOBUKI_I_GAIN = 129
    PROPERTY_DOUBLE_KOBUKI_D_GAIN = 130
    PROPERTY_DOUBLE_QARM_P_GAIN_YAW = 128
    PROPERTY_DOUBLE_QARM_P_GAIN_SHOULDER = 129
    PROPERTY_DOUBLE_QARM_P_GAIN_ELBOW = 130
    PROPERTY_DOUBLE_QARM_P_GAIN_WRIST = 131
    PROPERTY_DOUBLE_QARM_P_GAIN_GRIPPER = 132
    PROPERTY_DOUBLE_QARM_I_GAIN_YAW = 133
    PROPERTY_DOUBLE_QARM_I_GAIN_SHOULDER = 134
    PROPERTY_DOUBLE_QARM_I_GAIN_ELBOW = 135
    PROPERTY_DOUBLE_QARM_I_GAIN_WRIST = 136
    PROPERTY_DOUBLE_QARM_I_GAIN_GRIPPER = 137
    PROPERTY_DOUBLE_QARM_D_GAIN_YAW = 138
    PROPERTY_DOUBLE_QARM_D_GAIN_SHOULDER = 139
    PROPERTY_DOUBLE_QARM_D_GAIN_ELBOW = 140
    PROPERTY_DOUBLE_QARM_D_GAIN_WRIST = 141
    PROPERTY_DOUBLE_QARM_D_GAIN_GRIPPER = 142
    PROPERTY_DOUBLE_QARM_FF_VEL_GAIN_YAW = 143
    PROPERTY_DOUBLE_QARM_FF_VEL_GAIN_SHOULDER = 144
    PROPERTY_DOUBLE_QARM_FF_VEL_GAIN_ELBOW = 145
    PROPERTY_DOUBLE_QARM_FF_VEL_GAIN_WRIST = 146
    PROPERTY_DOUBLE_QARM_FF_VEL_GAIN_GRIPPER = 147
    PROPERTY_DOUBLE_QARM_FF_ACC_GAIN_YAW = 148
    PROPERTY_DOUBLE_QARM_FF_ACC_GAIN_SHOULDER = 149
    PROPERTY_DOUBLE_QARM_FF_ACC_GAIN_ELBOW = 150
    PROPERTY_DOUBLE_QARM_FF_ACC_GAIN_WRIST = 151
    PROPERTY_DOUBLE_QARM_FF_ACC_GAIN_GRIPPER = 152
end

const t_hil_double_property = tag_hil_double_property

@cenum tag_other_channel_assignments::UInt32 begin
    OTHER_CHANNELS_LINEAR_POSITIONS = 0
    OTHER_CHANNELS_ANGULAR_POSITIONS = 1000
    OTHER_CHANNELS_LINEAR_VELOCITIES = 2000
    OTHER_CHANNELS_ANGULAR_VELOCITIES = 3000
    OTHER_CHANNELS_LINEAR_ACCELERATIONS = 4000
    OTHER_CHANNELS_ANGULAR_ACCELERATIONS = 5000
    OTHER_CHANNELS_FORCES = 6000
    OTHER_CHANNELS_TORQUES = 7000
    OTHER_CHANNELS_MAGNETIC_FIELDS = 8000
    OTHER_CHANNELS_PRESSURE = 9000
    OTHER_CHANNELS_TEMPERATURE = 10000
    OTHER_CHANNELS_OPERATING_CAPACITY = 11000
    OTHER_CHANNELS_TIME = 12000
    OTHER_CHANNELS_COUNTS = 13000
    OTHER_CHANNELS_COUNTS_PER_SECOND = 14000
    OTHER_CHANNELS_COUNTS_PER_SECOND_2 = 15000
    OTHER_CHANNELS_ENUMERATION = 16000
    OTHER_CHANNELS_RAW_DATA = 17000
    OTHER_CHANNELS_CALIBRATION_DATA = 18000
    OTHER_CHANNELS_RESISTANCE = 19000
end

@cenum tag_interrupt_source_assignments::UInt32 begin
    INTERRUPT_SOURCE_DIGITAL_INPUT = 0
    INTERRUPT_SOURCE_ENCODER_INDEX_PULSE = 1000
    INTERRUPT_SOURCE_ANALOG_THRESHOLD = 2000
    INTERRUPT_SOURCE_TIMER = 3000
    INTERRUPT_SOURCE_ERROR = 4000
    INTERRUPT_SOURCE_ERROR_ESTOP = 4000
    INTERRUPT_SOURCE_ERROR_MEMORY_FAILURE = 4100
    INTERRUPT_SOURCE_ERROR_OSCILLATOR_FAILURE = 4101
    INTERRUPT_SOURCE_ERROR_STACK_OVERFLOW = 4102
    INTERRUPT_SOURCE_ERROR_PAGE_FAULT = 4103
    INTERRUPT_SOURCE_ERROR_DIVIDE_BY_ZERO = 4104
    INTERRUPT_SOURCE_ERROR_NUMERIC_OVERFLOW = 4105
    INTERRUPT_SOURCE_ERROR_LOW_VOLTAGE = 4200
    INTERRUPT_SOURCE_ERROR_OVER_VOLTAGE = 4300
    INTERRUPT_SOURCE_ERROR_NOISE = 4400
    INTERRUPT_SOURCE_USER = 30000
end

@cenum tag_buffer_overflow_mode::UInt32 begin
    BUFFER_MODE_ERROR_ON_OVERFLOW = 0
    BUFFER_MODE_OVERWRITE_ON_OVERFLOW = 1
    BUFFER_MODE_DISCARD_ON_OVERFLOW = 2
    BUFFER_MODE_WAIT_ON_OVERFLOW = 3
    BUFFER_MODE_SYNCHRONIZE = 4
    NUMBER_OF_BUFFER_OVERFLOW_MODES = 5
end

const t_buffer_overflow_mode = tag_buffer_overflow_mode

@cenum tag_esc_protocol::UInt32 begin
    ESC_PROTOCOL_INVALID = 0
    ESC_PROTOCOL_STANDARD_PWM = 1
    ESC_PROTOCOL_ONESHOT125 = 2
    ESC_PROTOCOL_ONESHOT42 = 3
    ESC_PROTOCOL_MULTISHOT = 4
    ESC_PROTOCOL_DSHOT = 5
    NUMBER_OF_ESC_PROTOCOLS = 6
end

const t_esc_protocol = tag_esc_protocol

@cenum tag_dshot_command::UInt32 begin
    DSHOT_COMMAND_DISARM = 0
    DSHOT_COMMAND_BEEP_LOW = 1
    DSHOT_COMMAND_BEEP_MEDIUM_LOW = 2
    DSHOT_COMMAND_BEEP_MEDIUM = 3
    DSHOT_COMMAND_BEEP_MEDIUM_HIGH = 4
    DSHOT_COMMAND_BEEP_HIGH = 5
    DSHOT_COMMAND_ESC_INFORMATION = 6
    DSHOT_COMMAND_ROTATE = 7
    DSHOT_COMMAND_ROTATE_OTHER = 8
    DSHOT_COMMAND_3D_MODE_OFF = 9
    DSHOT_COMMAND_3D_MODE_ON = 10
    DSHOT_COMMAND_ESC_SETTINGS = 11
    DSHOT_COMMAND_SAVE_SETTINGS = 12
    DSHOT_COMMAND_ROTATE_NORMAL = 20
    DSHOT_COMMAND_ROTATE_REVERSE = 21
    DSHOT_COMMAND_LED0_ON = 22
    DSHOT_COMMAND_LED1_ON = 23
    DSHOT_COMMAND_LED2_ON = 24
    DSHOT_COMMAND_LED3_ON = 25
    DSHOT_COMMAND_LED0_OFF = 26
    DSHOT_COMMAND_LED1_OFF = 27
    DSHOT_COMMAND_LED2_OFF = 28
    DSHOT_COMMAND_LED3_OFF = 29
    DSHOT_COMMAND_AUDIO_ON_OFF = 30
    DSHOT_COMMAND_SILENT_MODE = 31
    DSHOT_COMMAND_ARMED = 48
end

const t_dshot_command = tag_dshot_command

const t_uint = Cuint

const t_uint32 = t_uint

const t_ushort = Cushort

const t_uint16 = t_ushort

struct tag_version
    size::t_uint32
    major::t_uint16
    minor::t_uint16
    release::t_uint16
    build::t_uint16
end

const t_version = tag_version

function hil_get_version(version)
    ccall((:hil_get_version, hil_sdk), t_error, (Ptr{t_version},), version)
end

function hil_wopen(card_type, card_identifier, card)
    ccall((:hil_wopen, hil_sdk), t_error, (Ptr{Cwchar_t}, Ptr{Cwchar_t}, Ptr{t_card}), card_type, card_identifier, card)
end

const t_boolean = Cchar

function hil_is_valid(card)
    ccall((:hil_is_valid, hil_sdk), t_boolean, (t_card,), card)
end

function hil_close(card)
    ccall((:hil_close, hil_sdk), t_error, (t_card,), card)
end

function hil_close_all()
    ccall((:hil_close_all, hil_sdk), t_error, ())
end

function hil_acquire_exclusive_access(card, card_specific)
    ccall((:hil_acquire_exclusive_access, hil_sdk), t_error, (t_card, Ptr{Cvoid}), card, card_specific)
end

function hil_release_exclusive_access(card, card_specific)
    ccall((:hil_release_exclusive_access, hil_sdk), t_error, (t_card, Ptr{Cvoid}), card, card_specific)
end

const t_double = Cdouble

function hil_set_analog_input_ranges(card, analog_channels, num_channels, minimums, maximums)
    ccall((:hil_set_analog_input_ranges, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}), card, analog_channels, num_channels, minimums, maximums)
end

function hil_set_analog_input_configuration(card, analog_channels, num_channels, config)
    ccall((:hil_set_analog_input_configuration, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_analog_input_configuration}), card, analog_channels, num_channels, config)
end

function hil_set_analog_output_ranges(card, analog_channels, num_channels, minimums, maximums)
    ccall((:hil_set_analog_output_ranges, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}), card, analog_channels, num_channels, minimums, maximums)
end

const t_int32 = t_int

function hil_set_encoder_counts(card, encoder_channels, num_channels, buffer)
    ccall((:hil_set_encoder_counts, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_int32}), card, encoder_channels, num_channels, buffer)
end

function hil_set_encoder_quadrature_mode(card, encoder_channels, num_channels, mode)
    ccall((:hil_set_encoder_quadrature_mode, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_encoder_quadrature_mode}), card, encoder_channels, num_channels, mode)
end

function hil_set_encoder_filter_frequency(card, encoder_channels, num_channels, frequency)
    ccall((:hil_set_encoder_filter_frequency, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, encoder_channels, num_channels, frequency)
end

function hil_set_digital_directions(card, digital_inputs, num_digital_inputs, digital_outputs, num_digital_outputs)
    ccall((:hil_set_digital_directions, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32), card, digital_inputs, num_digital_inputs, digital_outputs, num_digital_outputs)
end

function hil_set_digital_output_configuration(card, channels, num_channels, configurations)
    ccall((:hil_set_digital_output_configuration, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_digital_configuration}), card, channels, num_channels, configurations)
end

function hil_set_clock_mode(card, clocks, num_clocks, modes)
    ccall((:hil_set_clock_mode, hil_sdk), t_error, (t_card, Ptr{t_clock}, t_uint32, Ptr{t_clock_mode}), card, clocks, num_clocks, modes)
end

function hil_set_clock_frequency(card, channels, num_channels, frequency)
    ccall((:hil_set_clock_frequency, hil_sdk), t_error, (t_card, Ptr{t_clock}, t_uint32, Ptr{t_double}), card, channels, num_channels, frequency)
end

function hil_set_pwm_mode(card, pwm_channels, num_channels, mode)
    ccall((:hil_set_pwm_mode, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_pwm_mode}), card, pwm_channels, num_channels, mode)
end

function hil_set_pwm_configuration(card, pwm_channels, num_channels, configurations, alignments, polarities)
    ccall((:hil_set_pwm_configuration, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_pwm_configuration}, Ptr{t_pwm_alignment}, Ptr{t_pwm_polarity}), card, pwm_channels, num_channels, configurations, alignments, polarities)
end

function hil_set_pwm_deadband(card, pwm_channels, num_channels, leading_edge_deadband, trailing_edge_deadband)
    ccall((:hil_set_pwm_deadband, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}), card, pwm_channels, num_channels, leading_edge_deadband, trailing_edge_deadband)
end

function hil_set_pwm_frequency(card, pwm_channels, num_channels, frequency)
    ccall((:hil_set_pwm_frequency, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, pwm_channels, num_channels, frequency)
end

function hil_set_pwm_duty_cycle(card, pwm_channels, num_channels, duty_cycle)
    ccall((:hil_set_pwm_duty_cycle, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, pwm_channels, num_channels, duty_cycle)
end

function hil_set_card_specific_woptions(card, options, options_size)
    ccall((:hil_set_card_specific_woptions, hil_sdk), t_error, (t_card, Ptr{Cwchar_t}, Csize_t), card, options, options_size)
end

function hil_get_integer_property(card, properties, num_properties, buffer)
    ccall((:hil_get_integer_property, hil_sdk), t_error, (t_card, Ptr{t_hil_integer_property}, t_uint, Ptr{t_int32}), card, properties, num_properties, buffer)
end

function hil_get_double_property(card, properties, num_properties, buffer)
    ccall((:hil_get_double_property, hil_sdk), t_error, (t_card, Ptr{t_hil_double_property}, t_uint, Ptr{t_double}), card, properties, num_properties, buffer)
end

function hil_get_wstring_property(card, property_code, buffer, buffer_size)
    ccall((:hil_get_wstring_property, hil_sdk), t_error, (t_card, t_hil_string_property, Ptr{Cwchar_t}, Csize_t), card, property_code, buffer, buffer_size)
end

function hil_set_integer_property(card, properties, num_properties, buffer)
    ccall((:hil_set_integer_property, hil_sdk), t_error, (t_card, Ptr{t_hil_integer_property}, t_uint, Ptr{t_int32}), card, properties, num_properties, buffer)
end

function hil_set_double_property(card, properties, num_properties, buffer)
    ccall((:hil_set_double_property, hil_sdk), t_error, (t_card, Ptr{t_hil_double_property}, t_uint, Ptr{t_double}), card, properties, num_properties, buffer)
end

function hil_set_wstring_property(card, property_code, buffer, buffer_size)
    ccall((:hil_set_wstring_property, hil_sdk), t_error, (t_card, t_hil_string_property, Ptr{Cwchar_t}, Csize_t), card, property_code, buffer, buffer_size)
end

function hil_read_analog(card, analog_channels, num_channels, buffer)
    ccall((:hil_read_analog, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, analog_channels, num_channels, buffer)
end

function hil_read_encoder(card, encoder_channels, num_channels, buffer)
    ccall((:hil_read_encoder, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_int32}), card, encoder_channels, num_channels, buffer)
end

function hil_read_digital(card, digital_lines, num_lines, buffer)
    ccall((:hil_read_digital, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_boolean}), card, digital_lines, num_lines, buffer)
end

function hil_read_other(card, other_channels, num_channels, buffer)
    ccall((:hil_read_other, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, other_channels, num_channels, buffer)
end

function hil_read_analog_codes(card, analog_channels, num_channels, buffer)
    ccall((:hil_read_analog_codes, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_int32}), card, analog_channels, num_channels, buffer)
end

function hil_read(card, analog_channels, num_analog_channels, encoder_channels, num_encoder_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, analog_buffer, encoder_buffer, digital_buffer, other_buffer)
    ccall((:hil_read, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_int32}, Ptr{t_boolean}, Ptr{t_double}), card, analog_channels, num_analog_channels, encoder_channels, num_encoder_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, analog_buffer, encoder_buffer, digital_buffer, other_buffer)
end

function hil_write_analog(card, analog_channels, num_channels, buffer)
    ccall((:hil_write_analog, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, analog_channels, num_channels, buffer)
end

function hil_esc_output(protocol, throttle, telemetry, command, pwm_output)
    ccall((:hil_esc_output, hil_sdk), t_error, (t_esc_protocol, t_double, t_boolean, t_dshot_command, Ptr{t_double}), protocol, throttle, telemetry, command, pwm_output)
end

function hil_write_pwm(card, pwm_channels, num_channels, buffer)
    ccall((:hil_write_pwm, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, pwm_channels, num_channels, buffer)
end

function hil_write_digital(card, digital_lines, num_lines, buffer)
    ccall((:hil_write_digital, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_boolean}), card, digital_lines, num_lines, buffer)
end

function hil_write_other(card, other_channels, num_channels, buffer)
    ccall((:hil_write_other, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, other_channels, num_channels, buffer)
end

function hil_write_analog_codes(card, analog_channels, num_channels, buffer)
    ccall((:hil_write_analog_codes, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_int32}), card, analog_channels, num_channels, buffer)
end

function hil_write(card, analog_channels, num_analog_channels, pwm_channels, num_pwm_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, analog_buffer, pwm_buffer, digital_buffer, other_buffer)
    ccall((:hil_write, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}, Ptr{t_boolean}, Ptr{t_double}), card, analog_channels, num_analog_channels, pwm_channels, num_pwm_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, analog_buffer, pwm_buffer, digital_buffer, other_buffer)
end

function hil_read_analog_write_analog(card, analog_input_channels, num_analog_input_channels, analog_output_channels, num_analog_output_channels, analog_input_buffer, analog_output_buffer)
    ccall((:hil_read_analog_write_analog, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}), card, analog_input_channels, num_analog_input_channels, analog_output_channels, num_analog_output_channels, analog_input_buffer, analog_output_buffer)
end

function hil_read_encoder_write_pwm(card, encoder_input_channels, num_encoder_input_channels, pwm_output_channels, num_pwm_output_channels, encoder_input_buffer, pwm_output_buffer)
    ccall((:hil_read_encoder_write_pwm, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_int32}, Ptr{t_double}), card, encoder_input_channels, num_encoder_input_channels, pwm_output_channels, num_pwm_output_channels, encoder_input_buffer, pwm_output_buffer)
end

function hil_read_digital_write_digital(card, digital_input_lines, num_digital_input_lines, digital_output_lines, num_digital_output_lines, digital_input_buffer, digital_output_buffer)
    ccall((:hil_read_digital_write_digital, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_boolean}, Ptr{t_boolean}), card, digital_input_lines, num_digital_input_lines, digital_output_lines, num_digital_output_lines, digital_input_buffer, digital_output_buffer)
end

function hil_read_other_write_other(card, other_input_channels, num_other_input_channels, other_output_channels, num_other_output_channels, other_input_buffer, other_output_buffer)
    ccall((:hil_read_other_write_other, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}), card, other_input_channels, num_other_input_channels, other_output_channels, num_other_output_channels, other_input_buffer, other_output_buffer)
end

function hil_read_write(card, analog_input_channels, num_analog_input_channels, encoder_input_channels, num_encoder_input_channels, digital_input_lines, num_digital_input_lines, other_input_channels, num_other_input_channels, analog_output_channels, num_analog_output_channels, pwm_output_channels, num_pwm_output_channels, digital_output_lines, num_digital_output_lines, other_output_channels, num_other_output_channels, analog_input_buffer, encoder_input_buffer, digital_input_buffer, other_input_buffer, analog_output_buffer, pwm_output_buffer, digital_output_buffer, other_output_buffer)
    ccall((:hil_read_write, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_int32}, Ptr{t_boolean}, Ptr{t_double}, Ptr{t_double}, Ptr{t_double}, Ptr{t_boolean}, Ptr{t_double}), card, analog_input_channels, num_analog_input_channels, encoder_input_channels, num_encoder_input_channels, digital_input_lines, num_digital_input_lines, other_input_channels, num_other_input_channels, analog_output_channels, num_analog_output_channels, pwm_output_channels, num_pwm_output_channels, digital_output_lines, num_digital_output_lines, other_output_channels, num_other_output_channels, analog_input_buffer, encoder_input_buffer, digital_input_buffer, other_input_buffer, analog_output_buffer, pwm_output_buffer, digital_output_buffer, other_output_buffer)
end

function hil_read_analog_buffer(card, clock, frequency, num_samples, analog_channels, num_channels, buffer)
    ccall((:hil_read_analog_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, clock, frequency, num_samples, analog_channels, num_channels, buffer)
end

function hil_read_encoder_buffer(card, clock, frequency, num_samples, encoder_channels, num_channels, buffer)
    ccall((:hil_read_encoder_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_int32}), card, clock, frequency, num_samples, encoder_channels, num_channels, buffer)
end

function hil_read_digital_buffer(card, clock, frequency, num_samples, digital_lines, num_lines, buffer)
    ccall((:hil_read_digital_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_boolean}), card, clock, frequency, num_samples, digital_lines, num_lines, buffer)
end

function hil_read_other_buffer(card, clock, frequency, num_samples, other_channels, num_channels, buffer)
    ccall((:hil_read_other_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, clock, frequency, num_samples, other_channels, num_channels, buffer)
end

function hil_read_buffer(card, clock, frequency, num_samples, analog_channels, num_analog_channels, encoder_channels, num_encoder_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, analog_buffer, encoder_buffer, digital_buffer, other_buffer)
    ccall((:hil_read_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_int32}, Ptr{t_boolean}, Ptr{t_double}), card, clock, frequency, num_samples, analog_channels, num_analog_channels, encoder_channels, num_encoder_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, analog_buffer, encoder_buffer, digital_buffer, other_buffer)
end

function hil_write_analog_buffer(card, clock, frequency, num_samples, analog_channels, num_channels, buffer)
    ccall((:hil_write_analog_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, clock, frequency, num_samples, analog_channels, num_channels, buffer)
end

function hil_write_pwm_buffer(card, clock, frequency, num_samples, pwm_channels, num_channels, buffer)
    ccall((:hil_write_pwm_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, clock, frequency, num_samples, pwm_channels, num_channels, buffer)
end

function hil_write_digital_buffer(card, clock, frequency, num_samples, digital_lines, num_lines, buffer)
    ccall((:hil_write_digital_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_boolean}), card, clock, frequency, num_samples, digital_lines, num_lines, buffer)
end

function hil_write_other_buffer(card, clock, frequency, num_samples, other_channels, num_channels, buffer)
    ccall((:hil_write_other_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, clock, frequency, num_samples, other_channels, num_channels, buffer)
end

function hil_write_buffer(card, clock, frequency, num_samples, analog_channels, num_analog_channels, pwm_channels, num_pwm_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, analog_buffer, pwm_buffer, digital_buffer, other_buffer)
    ccall((:hil_write_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}, Ptr{t_boolean}, Ptr{t_double}), card, clock, frequency, num_samples, analog_channels, num_analog_channels, pwm_channels, num_pwm_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, analog_buffer, pwm_buffer, digital_buffer, other_buffer)
end

function hil_read_analog_write_analog_buffer(card, clock, frequency, num_samples, analog_input_channels, num_analog_input_channels, analog_output_channels, num_analog_output_channels, analog_input_buffer, analog_output_buffer)
    ccall((:hil_read_analog_write_analog_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}), card, clock, frequency, num_samples, analog_input_channels, num_analog_input_channels, analog_output_channels, num_analog_output_channels, analog_input_buffer, analog_output_buffer)
end

function hil_read_encoder_write_pwm_buffer(card, clock, frequency, num_samples, encoder_input_channels, num_encoder_input_channels, pwm_output_channels, num_pwm_output_channels, encoder_input_buffer, pwm_output_buffer)
    ccall((:hil_read_encoder_write_pwm_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_int32}, Ptr{t_double}), card, clock, frequency, num_samples, encoder_input_channels, num_encoder_input_channels, pwm_output_channels, num_pwm_output_channels, encoder_input_buffer, pwm_output_buffer)
end

function hil_read_digital_write_digital_buffer(card, clock, frequency, num_samples, digital_input_lines, num_digital_input_lines, digital_output_lines, num_digital_output_lines, digital_input_buffer, digital_output_buffer)
    ccall((:hil_read_digital_write_digital_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_boolean}, Ptr{t_boolean}), card, clock, frequency, num_samples, digital_input_lines, num_digital_input_lines, digital_output_lines, num_digital_output_lines, digital_input_buffer, digital_output_buffer)
end

function hil_read_other_write_other_buffer(card, clock, frequency, num_samples, other_input_channels, num_other_input_channels, other_output_channels, num_other_output_channels, other_input_buffer, other_output_buffer)
    ccall((:hil_read_other_write_other_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_double}), card, clock, frequency, num_samples, other_input_channels, num_other_input_channels, other_output_channels, num_other_output_channels, other_input_buffer, other_output_buffer)
end

function hil_read_write_buffer(card, clock, frequency, num_samples, analog_input_channels, num_analog_input_channels, encoder_input_channels, num_encoder_input_channels, digital_input_lines, num_digital_input_lines, other_input_channels, num_other_input_channels, analog_output_channels, num_analog_output_channels, pwm_output_channels, num_pwm_output_channels, digital_output_lines, num_digital_output_lines, other_output_channels, num_other_output_channels, analog_input_buffer, encoder_input_buffer, digital_input_buffer, other_input_buffer, analog_output_buffer, pwm_output_buffer, digital_output_buffer, other_output_buffer)
    ccall((:hil_read_write_buffer, hil_sdk), t_error, (t_card, t_clock, t_double, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_double}, Ptr{t_int32}, Ptr{t_boolean}, Ptr{t_double}, Ptr{t_double}, Ptr{t_double}, Ptr{t_boolean}, Ptr{t_double}), card, clock, frequency, num_samples, analog_input_channels, num_analog_input_channels, encoder_input_channels, num_encoder_input_channels, digital_input_lines, num_digital_input_lines, other_input_channels, num_other_input_channels, analog_output_channels, num_analog_output_channels, pwm_output_channels, num_pwm_output_channels, digital_output_lines, num_digital_output_lines, other_output_channels, num_other_output_channels, analog_input_buffer, encoder_input_buffer, digital_input_buffer, other_input_buffer, analog_output_buffer, pwm_output_buffer, digital_output_buffer, other_output_buffer)
end

function hil_task_create_analog_reader(card, samples_in_buffer, analog_channels, num_analog_channels, task)
    ccall((:hil_task_create_analog_reader, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, analog_channels, num_analog_channels, task)
end

function hil_task_create_encoder_reader(card, samples_in_buffer, encoder_channels, num_encoder_channels, task)
    ccall((:hil_task_create_encoder_reader, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, encoder_channels, num_encoder_channels, task)
end

function hil_task_create_digital_reader(card, samples_in_buffer, digital_lines, num_digital_lines, task)
    ccall((:hil_task_create_digital_reader, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, digital_lines, num_digital_lines, task)
end

function hil_task_create_other_reader(card, samples_in_buffer, other_channels, num_other_channels, task)
    ccall((:hil_task_create_other_reader, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, other_channels, num_other_channels, task)
end

function hil_task_create_reader(card, samples_in_buffer, analog_channels, num_analog_channels, encoder_channels, num_encoder_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, task)
    ccall((:hil_task_create_reader, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, analog_channels, num_analog_channels, encoder_channels, num_encoder_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, task)
end

function hil_task_create_analog_writer(card, samples_in_buffer, analog_channels, num_analog_channels, task)
    ccall((:hil_task_create_analog_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, analog_channels, num_analog_channels, task)
end

function hil_task_create_pwm_writer(card, samples_in_buffer, pwm_channels, num_pwm_channels, task)
    ccall((:hil_task_create_pwm_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, pwm_channels, num_pwm_channels, task)
end

function hil_task_create_digital_writer(card, samples_in_buffer, digital_lines, num_digital_lines, task)
    ccall((:hil_task_create_digital_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, digital_lines, num_digital_lines, task)
end

function hil_task_create_other_writer(card, samples_in_buffer, other_channels, num_other_channels, task)
    ccall((:hil_task_create_other_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, other_channels, num_other_channels, task)
end

function hil_task_create_writer(card, samples_in_buffer, analog_channels, num_analog_channels, pwm_channels, num_pwm_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, task)
    ccall((:hil_task_create_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, analog_channels, num_analog_channels, pwm_channels, num_pwm_channels, digital_lines, num_digital_lines, other_channels, num_other_channels, task)
end

function hil_task_create_analog_reader_analog_writer(card, samples_in_buffer, analog_input_channels, num_analog_input_channels, analog_output_channels, num_analog_output_channels, task)
    ccall((:hil_task_create_analog_reader_analog_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, analog_input_channels, num_analog_input_channels, analog_output_channels, num_analog_output_channels, task)
end

function hil_task_create_encoder_reader_pwm_writer(card, samples_in_buffer, encoder_input_channels, num_encoder_input_channels, pwm_output_channels, num_pwm_output_channels, task)
    ccall((:hil_task_create_encoder_reader_pwm_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, encoder_input_channels, num_encoder_input_channels, pwm_output_channels, num_pwm_output_channels, task)
end

function hil_task_create_digital_reader_digital_writer(card, samples_in_buffer, digital_input_lines, num_digital_input_lines, digital_output_lines, num_digital_output_lines, task)
    ccall((:hil_task_create_digital_reader_digital_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, digital_input_lines, num_digital_input_lines, digital_output_lines, num_digital_output_lines, task)
end

function hil_task_create_other_reader_other_writer(card, samples_in_buffer, other_input_channels, num_other_input_channels, other_output_channels, num_other_output_channels, task)
    ccall((:hil_task_create_other_reader_other_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, other_input_channels, num_other_input_channels, other_output_channels, num_other_output_channels, task)
end

function hil_task_create_reader_writer(card, samples_in_buffer, analog_input_channels, num_analog_input_channels, encoder_input_channels, num_encoder_input_channels, digital_input_lines, num_digital_input_lines, other_input_channels, num_other_input_channels, analog_output_channels, num_analog_output_channels, pwm_output_channels, num_pwm_output_channels, digital_output_lines, num_digital_output_lines, other_output_channels, num_other_output_channels, task)
    ccall((:hil_task_create_reader_writer, hil_sdk), t_error, (t_card, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_uint32}, t_uint32, Ptr{t_task}), card, samples_in_buffer, analog_input_channels, num_analog_input_channels, encoder_input_channels, num_encoder_input_channels, digital_input_lines, num_digital_input_lines, other_input_channels, num_other_input_channels, analog_output_channels, num_analog_output_channels, pwm_output_channels, num_pwm_output_channels, digital_output_lines, num_digital_output_lines, other_output_channels, num_other_output_channels, task)
end

function hil_task_set_buffer_overflow_mode(task, mode)
    ccall((:hil_task_set_buffer_overflow_mode, hil_sdk), t_error, (t_task, t_buffer_overflow_mode), task, mode)
end

function hil_task_get_buffer_overflows(task)
    ccall((:hil_task_get_buffer_overflows, hil_sdk), t_int, (t_task,), task)
end

function hil_task_get_card(task, card)
    ccall((:hil_task_get_card, hil_sdk), t_error, (t_task, Ptr{t_card}), task, card)
end

function hil_task_get_num_channels(task, num_analog_input_channels, num_encoder_input_channels, num_digital_input_lines, num_other_input_channels, num_analog_output_channels, num_pwm_output_channels, num_digital_output_lines, num_other_output_channels)
    ccall((:hil_task_get_num_channels, hil_sdk), t_error, (t_task, Ptr{t_uint32}, Ptr{t_uint32}, Ptr{t_uint32}, Ptr{t_uint32}, Ptr{t_uint32}, Ptr{t_uint32}, Ptr{t_uint32}, Ptr{t_uint32}), task, num_analog_input_channels, num_encoder_input_channels, num_digital_input_lines, num_other_input_channels, num_analog_output_channels, num_pwm_output_channels, num_digital_output_lines, num_other_output_channels)
end

function hil_task_start(task, clock, frequency, num_samples)
    ccall((:hil_task_start, hil_sdk), t_error, (t_task, t_clock, t_double, t_uint32), task, clock, frequency, num_samples)
end

function hil_task_read_analog(task, num_samples, analog_buffer)
    ccall((:hil_task_read_analog, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}), task, num_samples, analog_buffer)
end

function hil_task_read_encoder(task, num_samples, encoder_buffer)
    ccall((:hil_task_read_encoder, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_int32}), task, num_samples, encoder_buffer)
end

function hil_task_read_digital(task, num_samples, digital_buffer)
    ccall((:hil_task_read_digital, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_boolean}), task, num_samples, digital_buffer)
end

function hil_task_read_other(task, num_samples, other_buffer)
    ccall((:hil_task_read_other, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}), task, num_samples, other_buffer)
end

function hil_task_read(task, num_samples, analog_buffer, encoder_buffer, digital_buffer, other_buffer)
    ccall((:hil_task_read, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}, Ptr{t_int32}, Ptr{t_boolean}, Ptr{t_double}), task, num_samples, analog_buffer, encoder_buffer, digital_buffer, other_buffer)
end

function hil_task_write_analog(task, num_samples, analog_buffer)
    ccall((:hil_task_write_analog, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}), task, num_samples, analog_buffer)
end

function hil_task_write_pwm(task, num_samples, pwm_buffer)
    ccall((:hil_task_write_pwm, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}), task, num_samples, pwm_buffer)
end

function hil_task_write_digital(task, num_samples, digital_buffer)
    ccall((:hil_task_write_digital, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_boolean}), task, num_samples, digital_buffer)
end

function hil_task_write_other(task, num_samples, other_buffer)
    ccall((:hil_task_write_other, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}), task, num_samples, other_buffer)
end

function hil_task_write(task, num_samples, analog_buffer, pwm_buffer, digital_buffer, other_buffer)
    ccall((:hil_task_write, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}, Ptr{t_double}, Ptr{t_boolean}, Ptr{t_double}), task, num_samples, analog_buffer, pwm_buffer, digital_buffer, other_buffer)
end

function hil_task_read_analog_write_analog(task, num_samples, analog_input_buffer, analog_output_buffer)
    ccall((:hil_task_read_analog_write_analog, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}, Ptr{t_double}), task, num_samples, analog_input_buffer, analog_output_buffer)
end

function hil_task_read_encoder_write_pwm(task, num_samples, encoder_input_buffer, pwm_output_buffer)
    ccall((:hil_task_read_encoder_write_pwm, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_int32}, Ptr{t_double}), task, num_samples, encoder_input_buffer, pwm_output_buffer)
end

function hil_task_read_digital_write_digital(task, num_samples, digital_input_buffer, digital_output_buffer)
    ccall((:hil_task_read_digital_write_digital, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_boolean}, Ptr{t_boolean}), task, num_samples, digital_input_buffer, digital_output_buffer)
end

function hil_task_read_other_write_other(task, num_samples, other_input_buffer, other_output_buffer)
    ccall((:hil_task_read_other_write_other, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}, Ptr{t_double}), task, num_samples, other_input_buffer, other_output_buffer)
end

function hil_task_read_write(task, num_samples, analog_input_buffer, encoder_input_buffer, digital_input_buffer, other_input_buffer, analog_output_buffer, pwm_output_buffer, digital_output_buffer, other_output_buffer)
    ccall((:hil_task_read_write, hil_sdk), t_error, (t_task, t_uint32, Ptr{t_double}, Ptr{t_int32}, Ptr{t_boolean}, Ptr{t_double}, Ptr{t_double}, Ptr{t_double}, Ptr{t_boolean}, Ptr{t_double}), task, num_samples, analog_input_buffer, encoder_input_buffer, digital_input_buffer, other_input_buffer, analog_output_buffer, pwm_output_buffer, digital_output_buffer, other_output_buffer)
end

function hil_task_flush(task)
    ccall((:hil_task_flush, hil_sdk), t_error, (t_task,), task)
end

function hil_task_stop(task)
    ccall((:hil_task_stop, hil_sdk), t_error, (t_task,), task)
end

function hil_task_delete(task)
    ccall((:hil_task_delete, hil_sdk), t_error, (t_task,), task)
end

function hil_task_stop_all(card)
    ccall((:hil_task_stop_all, hil_sdk), t_error, (t_card,), card)
end

function hil_task_delete_all(card)
    ccall((:hil_task_delete_all, hil_sdk), t_error, (t_card,), card)
end

function hil_task_is_valid(task)
    ccall((:hil_task_is_valid, hil_sdk), t_boolean, (t_task,), task)
end

function hil_watchdog_set_analog_expiration_state(card, channels, num_channels, voltages)
    ccall((:hil_watchdog_set_analog_expiration_state, hil_sdk), t_error, (t_card, Ptr{t_uint}, t_uint, Ptr{t_double}), card, channels, num_channels, voltages)
end

function hil_watchdog_set_pwm_expiration_state(card, channels, num_channels, duty_cycles)
    ccall((:hil_watchdog_set_pwm_expiration_state, hil_sdk), t_error, (t_card, Ptr{t_uint}, t_uint, Ptr{t_double}), card, channels, num_channels, duty_cycles)
end

function hil_watchdog_set_digital_expiration_state(card, channels, num_channels, states)
    ccall((:hil_watchdog_set_digital_expiration_state, hil_sdk), t_error, (t_card, Ptr{t_uint}, t_uint, Ptr{t_digital_state}), card, channels, num_channels, states)
end

function hil_watchdog_set_other_expiration_state(card, channels, num_channels, values)
    ccall((:hil_watchdog_set_other_expiration_state, hil_sdk), t_error, (t_card, Ptr{t_uint}, t_uint, Ptr{t_double}), card, channels, num_channels, values)
end

function hil_watchdog_start(card, timeout)
    ccall((:hil_watchdog_start, hil_sdk), t_error, (t_card, t_double), card, timeout)
end

function hil_watchdog_reload(card)
    ccall((:hil_watchdog_reload, hil_sdk), t_error, (t_card,), card)
end

function hil_watchdog_is_expired(card)
    ccall((:hil_watchdog_is_expired, hil_sdk), t_error, (t_card,), card)
end

function hil_watchdog_clear(card)
    ccall((:hil_watchdog_clear, hil_sdk), t_error, (t_card,), card)
end

function hil_watchdog_stop(card)
    ccall((:hil_watchdog_stop, hil_sdk), t_error, (t_card,), card)
end

function hil_poll_interrupt(card, channels, num_channels, states)
    ccall((:hil_poll_interrupt, hil_sdk), t_int, (t_card, Ptr{t_uint}, t_uint, Ptr{t_boolean}), card, channels, num_channels, states)
end

function hil_monitor_create_interrupt_reader(card, channels, num_channels, monitor)
    ccall((:hil_monitor_create_interrupt_reader, hil_sdk), t_error, (t_card, Ptr{t_uint}, t_uint, Ptr{t_monitor}), card, channels, num_channels, monitor)
end

function hil_monitor_start(monitor)
    ccall((:hil_monitor_start, hil_sdk), t_error, (t_monitor,), monitor)
end

function hil_monitor_read_interrupt(monitor, states)
    ccall((:hil_monitor_read_interrupt, hil_sdk), t_int, (t_monitor, Ptr{t_boolean}), monitor, states)
end

function hil_monitor_stop(monitor)
    ccall((:hil_monitor_stop, hil_sdk), t_error, (t_monitor,), monitor)
end

function hil_monitor_delete(monitor)
    ccall((:hil_monitor_delete, hil_sdk), t_error, (t_monitor,), monitor)
end

function hil_monitor_stop_all(card)
    ccall((:hil_monitor_stop_all, hil_sdk), t_error, (t_card,), card)
end

function hil_monitor_delete_all(card)
    ccall((:hil_monitor_delete_all, hil_sdk), t_error, (t_card,), card)
end

function hil_monitor_is_valid(monitor)
    ccall((:hil_monitor_is_valid, hil_sdk), t_boolean, (t_monitor,), monitor)
end

function hil_set_analog_termination_state(card, analog_channels, num_channels, buffer)
    ccall((:hil_set_analog_termination_state, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, analog_channels, num_channels, buffer)
end

function hil_set_pwm_termination_state(card, pwm_channels, num_channels, buffer)
    ccall((:hil_set_pwm_termination_state, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, pwm_channels, num_channels, buffer)
end

function hil_set_digital_termination_state(card, digital_lines, num_lines, buffer)
    ccall((:hil_set_digital_termination_state, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_boolean}), card, digital_lines, num_lines, buffer)
end

function hil_set_other_termination_state(card, other_channels, num_channels, buffer)
    ccall((:hil_set_other_termination_state, hil_sdk), t_error, (t_card, Ptr{t_uint32}, t_uint32, Ptr{t_double}), card, other_channels, num_channels, buffer)
end

function hil_write_termination_states(card)
    ccall((:hil_write_termination_states, hil_sdk), t_error, (t_card,), card)
end

@cenum tag_error::UInt32 begin
    QERR_NO_ERROR = 0
    QERR_OUT_OF_MEMORY = 1
    QERR_OUT_OF_RESOURCES = 2
    QERR_OUT_OF_RANGE = 3
    QERR_INVALID_ARGUMENT = 4
    QERR_PAGE_FAULT = 5
    QERR_NOT_SUPPORTED = 6
    QERR_DEADLOCK = 7
    QERR_BUSY = 8
    QERR_OBJECT_NOT_FOUND = 9
    QERR_FILE_NOT_FOUND = 10
    QERR_NO_PERMISSION = 11
    QERR_TOO_MANY_PROCESSES = 12
    QERR_UNRECOGNIZED_ERROR = 13
    QERR_TIMED_OUT = 14
    QERR_LIBRARY_NOT_FOUND = 15
    QERR_LIBRARY_NOT_CLOSED = 16
    QERR_STRING_TOO_SMALL = 17
    QERR_STRING_NOT_TERMINATED = 18
    QERR_INVALID_URI = 19
    QERR_URI_OPTION_TOO_LONG = 20
    QERR_MISSING_URI_OPTION_VALUE = 21
    QERR_INVALID_URI_OPTION_VALUE = 22
    QERR_INVALID_URI_PORT_VALUE = 23
    QERR_MISSING_FUNCTION = 24
    QERR_INVALID_CONNECTION = 25
    QERR_NON_BLOCKING_NOT_SUPPORTED = 26
    QERR_CANNOT_INITIALIZE_SOCKETS = 27
    QERR_NAGLE_NOT_SUPPORTED = 28
    QERR_INVALID_BUFFER_SIZE = 29
    QERR_SOCKET_NOT_REUSABLE = 30
    QERR_CANNOT_BIND_SOCKET = 31
    QERR_CANNOT_LISTEN = 32
    QERR_CANNOT_CONNECT = 33
    QERR_WOULD_BLOCK = 34
    QERR_INTERRUPTED = 35
    QERR_HOST_NOT_FOUND = 36
    QERR_INVALID_SOCKET = 37
    QERR_CANNOT_LINGER = 38
    QERR_CANNOT_ACCEPT_CONNECTION = 39
    QERR_CANNOT_SEND = 40
    QERR_CANNOT_RECEIVE = 41
    QERR_CANNOT_POLL = 42
    QERR_CANNOT_SHUTDOWN = 43
    QERR_CONNECTION_SHUTDOWN = 44
    QERR_CANNOT_CLOSE = 45
    QERR_CANNOT_GET_TIME = 46
    QERR_CANNOT_SUBTRACT_TIMEOUTS = 47
    QERR_CANNOT_ADD_TIMEOUTS = 48
    QERR_CANNOT_OBTAIN_LOCK = 49
    QERR_INVALID_RECONFIGURATION = 50
    QERR_BEGIN_CONTROL_NOT_CALLED = 51
    QERR_CANNOT_SWITCH = 52
    QERR_INVALID_BASE = 53
    QERR_INVALID_TIMER_EVENT = 54
    QERR_INVALID_TIMER_NOTIFICATION = 55
    QERR_INVALID_TIMER_RESOLUTION = 56
    QERR_INVALID_TIMER_PERIOD = 57
    QERR_INVALID_TIMER = 58
    QERR_CANNOT_GET_RESOLUTION = 59
    QERR_CANNOT_SET_RESOLUTION = 60
    QERR_BEGIN_RESOLUTION_NOT_CALLED = 61
    QERR_STATE_INCOMPATIBLE = 62
    QERR_INVALID_STATE = 63
    QERR_INVALID_STREAM = 64
    QERR_STREAM_BUFFER_TOO_SMALL = 65
    QERR_INVALID_DPC = 66
    QERR_INVALID_METHOD = 67
    QERR_INVALID_INTERFACE = 68
    QERR_DPC_DISCONNECTED = 69
    QERR_INVALID_LOG = 70
    QERR_INVALID_SEMAPHORE = 71
    QERR_INVALID_MUTEX = 72
    QERR_ARGUMENT_LIST_TOO_BIG = 73
    QERR_FILE_NOT_EXECUTABLE = 74
    QERR_TOO_MANY_LINKS = 75
    QERR_NAME_TOO_LONG = 76
    QERR_NOT_DIRECTORY = 77
    QERR_NAME_CONFLICT = 78
    QERR_INVALID_ESCAPE_SEQUENCE = 79
    QERR_INVALID_STOP_BITS = 80
    QERR_INVALID_FLOW_CONTROL = 81
    QERR_INVALID_PARITY = 82
    QERR_NOT_SOCK_STREAM = 83
    QERR_CANNOT_FIND_SOCKET_DRIVER = 84
    QERR_NETWORK_FAILED = 85
    QERR_SOCKETS_NOT_INITIALIZED = 86
    QERR_OPERATION_IN_PROGRESS = 87
    QERR_INVALID_CARD_HANDLE = 88
    QERR_VERSION_ARGUMENT_IS_NULL = 89
    QERR_INVALID_VERSION_ARGUMENT = 90
    QERR_DRIVER_MISSING_OPEN_OR_CLOSE = 91
    QERR_CARD_ARGUMENT_IS_NULL = 92
    QERR_CARD_TYPE_ARGUMENT_IS_NULL = 93
    QERR_FUNCTION_NOT_SUPPORTED = 94
    QERR_MISSING_ANALOG_INPUTS = 95
    QERR_MISSING_ANALOG_OUTPUTS = 96
    QERR_MISSING_ENCODER_INPUTS = 97
    QERR_MISSING_PWM_OUTPUTS = 98
    QERR_MISSING_DIGITAL_INPUTS = 99
    QERR_MISSING_DIGITAL_OUTPUTS = 100
    QERR_MISSING_OTHER_INPUTS = 101
    QERR_MISSING_OTHER_OUTPUTS = 102
    QERR_MISSING_CLOCK_INPUTS = 103
    QERR_TASK_ARGUMENT_IS_NULL = 104
    QERR_INVALID_TASK_HANDLE = 105
    QERR_BOARD_ARGUMENT_IS_NULL = 106
    QERR_UNABLE_TO_OPEN_DRIVER = 107
    QERR_BOARD_NOT_FOUND = 108
    QERR_BOARD_IN_USE = 109
    QERR_UNABLE_TO_LOCK_DEVICE = 110
    QERR_BUFFER_OVERFLOW = 111
    QERR_UNABLE_TO_CLOSE_DRIVER = 112
    QERR_INVALID_BOARD_HANDLE = 113
    QERR_OUT_OF_REQUIRED_SYSTEM_RESOURCES = 114
    QERR_DRIVER_INCOMPATIBLE_WITH_BOARD_DLL = 115
    QERR_INTERNAL_BUFFER_TOO_SMALL = 116
    QERR_ANALOG_RESOURCE_IN_USE = 117
    QERR_INVALID_BUFFER_HANDLE = 118
    QERR_ANALOG_INPUT_CHANNELS_NOT_SUPPORTED = 119
    QERR_TOO_MANY_ANALOG_INPUT_CHANNELS = 120
    QERR_INVALID_ANALOG_INPUT_CHANNEL = 121
    QERR_ENCODER_INPUT_CHANNELS_NOT_SUPPORTED = 122
    QERR_TOO_MANY_ENCODER_INPUT_CHANNELS = 123
    QERR_INVALID_ENCODER_INPUT_CHANNEL = 124
    QERR_DIGITAL_INPUT_CHANNELS_NOT_SUPPORTED = 125
    QERR_TOO_MANY_DIGITAL_INPUT_CHANNELS = 126
    QERR_INVALID_DIGITAL_INPUT_CHANNEL = 127
    QERR_OTHER_INPUT_CHANNELS_NOT_SUPPORTED = 128
    QERR_TOO_MANY_OTHER_INPUT_CHANNELS = 129
    QERR_INVALID_OTHER_INPUT_CHANNEL = 130
    QERR_ANALOG_OUTPUT_CHANNELS_NOT_SUPPORTED = 131
    QERR_TOO_MANY_ANALOG_OUTPUT_CHANNELS = 132
    QERR_INVALID_ANALOG_OUTPUT_CHANNEL = 133
    QERR_PWM_OUTPUT_CHANNELS_NOT_SUPPORTED = 134
    QERR_TOO_MANY_PWM_OUTPUT_CHANNELS = 135
    QERR_INVALID_PWM_OUTPUT_CHANNEL = 136
    QERR_DIGITAL_OUTPUT_CHANNELS_NOT_SUPPORTED = 137
    QERR_TOO_MANY_DIGITAL_OUTPUT_CHANNELS = 138
    QERR_INVALID_DIGITAL_OUTPUT_CHANNEL = 139
    QERR_OTHER_OUTPUT_CHANNELS_NOT_SUPPORTED = 140
    QERR_TOO_MANY_OTHER_OUTPUT_CHANNELS = 141
    QERR_INVALID_OTHER_OUTPUT_CHANNEL = 142
    QERR_CONFLICTING_DIGITAL_DIRECTIONS = 143
    QERR_CLOCK_NOT_SUPPORTED = 144
    QERR_HARDWARE_CLOCK_IN_USE = 145
    QERR_TOO_MANY_CLOCKS = 146
    QERR_CLOCK_MODE_NOT_SUPPORTED = 147
    QERR_PWM_MODE_NOT_SUPPORTED = 148
    QERR_CLOCK_FREQUENCY_NOT_POSITIVE = 149
    QERR_CLOCK_FREQUENCY_TOO_HIGH = 150
    QERR_CLOCK_FREQUENCY_TOO_LOW = 151
    QERR_CLOCK_FREQUENCY_INVALID = 152
    QERR_DUTY_CYCLE_NOT_POSITIVE = 153
    QERR_DUTY_CYCLE_TOO_HIGH = 154
    QERR_WRONG_CLOCK_MODE = 155
    QERR_INVALID_OPERATION_HANDLE = 156
    QERR_OPERATION_ARGUMENT_IS_NULL = 157
    QERR_INTERRUPT_VECTOR_IN_USE = 158
    QERR_TOO_MANY_SAMPLES_FOR_BUFFER = 159
    QERR_MISSING_ANALOG_INPUT_BUFFER = 160
    QERR_MISSING_ENCODER_INPUT_BUFFER = 161
    QERR_MISSING_DIGITAL_INPUT_BUFFER = 162
    QERR_MISSING_OTHER_INPUT_BUFFER = 163
    QERR_MISSING_ANALOG_OUTPUT_BUFFER = 164
    QERR_MISSING_PWM_OUTPUT_BUFFER = 165
    QERR_MISSING_DIGITAL_OUTPUT_BUFFER = 166
    QERR_MISSING_OTHER_OUTPUT_BUFFER = 167
    QERR_READING_FROM_WRITE_ONLY_TASK = 168
    QERR_WRITING_TO_READ_ONLY_TASK = 169
    QERR_PROCESS_NOT_FOUND = 170
    QERR_PROCESS_CANNOT_BE_STOPPED = 171
    QERR_ERROR_MESSAGE_NOT_FOUND = 172
    QERR_PORT_IN_USE = 173
    QERR_HOST_BUSY = 174
    QERR_HOST_SHUTDOWN = 175
    QERR_CONNECTION_RESET = 176
    QERR_CHANNEL_NOT_LISTENING = 177
    QERR_CHANNEL_IS_LISTENING = 178
    QERR_UNRECOGNIZED_BOARD_TYPE = 179
    QERR_INVALID_PREFERENCES_ROOT = 180
    QERR_PREFERENCES_NODE_NOT_FOUND = 181
    QERR_CANNOT_ENUMERATE_VALUES = 182
    QERR_PREFERENCES_NODE_TOO_LONG = 183
    QERR_URI_NOT_FOUND = 184
    QERR_CANNOT_SET_PREFERENCES_VALUE = 185
    QERR_CANNOT_DELETE_PREFERENCES_VALUE = 186
    QERR_REMOVING_LAST_URI = 187
    QERR_REMOVING_URI_IN_USE = 188
    QERR_OPERATION_PENDING = 189
    QERR_OVERSAMPLING_DETECTED = 190
    QERR_TIMEBASE_ALREADY_REGISTERED = 191
    QERR_TIMEBASE_NOT_REGISTERED = 192
    QERR_CANNOT_GET_PREFERENCES_VALUE = 193
    QERR_INVALID_LICENSE = 194
    QERR_MISSING_LICENSE_FILE = 195
    QERR_ETHERCAT_MASTER_NOT_FOUND = 196
    QERR_CANNOT_OPEN_ETHERCAT = 197
    QERR_ETHERCAT_DEVICE_IS_NULL = 198
    QERR_ETHERCAT_SYNC_CLIENT_IS_NULL = 199
    QERR_INVALID_XML_COMMENT = 200
    QERR_INVALID_XML = 201
    QERR_INVALID_XML_DOCUMENT_TYPE = 202
    QERR_SPACE_PRECEDES_XML_DECLARATION = 203
    QERR_MULTIPLE_XML_ROOTS = 204
    QERR_UNTERMINATED_XML_COMMENT = 205
    QERR_MISSING_XML_VERSION = 206
    QERR_INVALID_XML_VERSION = 207
    QERR_INVALID_XML_ENCODING = 208
    QERR_INVALID_XML_STANDALONE = 209
    QERR_INVALID_XML_DECLARATION = 210
    QERR_INVALID_XML_DECLARATION_END = 211
    QERR_UNSUPPORTED_XML_MARKUP = 212
    QERR_MISSING_URI_PATH = 213
    QERR_INVALID_FILE_MODE = 214
    QERR_INVALID_FILE_SHARE_MODE = 215
    QERR_NO_FILE_SIZE = 216
    QERR_CHANGE_NOTIFICATIONS_NOT_SUPPORTED = 217
    QERR_WRITING_TO_READ_ONLY_STREAM = 218
    QERR_READING_FROM_WRITE_ONLY_STREAM = 219
    QERR_INVALID_STREAM_FORMAT = 220
    QERR_ILLEGAL_UTF8_CHARACTER = 221
    QERR_ILLEGAL_UTF16_CHARACTER = 222
    QERR_ILLEGAL_UTF32_CHARACTER = 223
    QERR_XML_DECLARATION_NOT_FIRST = 224
    QERR_XML_DOCTYPE_ALREADY_PARSED = 225
    QERR_INVALID_PI_TARGET_NAME = 226
    QERR_INVALID_XML_PROCESSING_INSTRUCTION = 227
    QERR_INVALID_XML_EXTERNAL_ID = 228
    QERR_INVALID_DOCTYPE_NAME = 229
    QERR_INVALID_XML_SYSTEM_LITERAL = 230
    QERR_INVALID_DOCTYPE_NOT_TERMINATED = 231
    QERR_INVALID_XML_ELEMENT_NAME = 232
    QERR_INVALID_XML_ELEMENT = 233
    QERR_MISSING_XML_ATTRIBUTE_VALUE = 234
    QERR_TAG_IN_XML_ATTRIBUTE_VALUES = 235
    QERR_INVALID_XML_ENTITY_REFERENCE = 236
    QERR_INVALID_XML_CHAR_REFERENCE = 237
    QERR_UNTERMINATED_XML_ATTRIBUTE_VALUE = 238
    QERR_CDATA_TERMINATOR_IN_CHAR_DATA = 239
    QERR_INVALID_XML_TAG = 240
    QERR_INVALID_XML_CDATA_TAG = 241
    QERR_INVALID_XML_CDATA = 242
    QERR_UNTERMINATED_XML_ELEMENT = 243
    QERR_INVALID_DOM_NODE = 244
    QERR_INVALID_DOM_NODE_LIST = 245
    QERR_ITEM_NOT_IN_LIST = 246
    QERR_STRING_IS_NULL = 247
    QERR_MISMATCHED_XML_ELEMENT_TAG = 248
    QERR_INVALID_DOM_NAMED_NODE_MAP = 249
    QERR_ITEM_NOT_IN_MAP = 250
    QERR_DUPLICATE_XML_ATTRIBUTE = 251
    QERR_ILLEGAL_UTF8_LEAD_CHAR = 252
    QERR_TRUNCATED_UTF8_CHAR = 253
    QERR_ILLEGAL_UTF16_LEAD_CHAR = 254
    QERR_TRUNCATED_UTF16_CHAR = 255
    QERR_CANNOT_START_LICENSE_MANAGER = 256
    QERR_CANNOT_STOP_LICENSE_MANAGER = 257
    QERR_TRUNCATED_UTF32_CHAR = 258
    QERR_INVALID_THREAD_AFFINITY = 259
    QERR_INVALID_PROCESS_AFFINITY = 260
    QERR_CANNOT_GET_PROCESS_AFFINITY = 261
    QERR_THREAD_AFFINITY_UNAVAILABLE = 262
    QERR_INCOMPLETE_WRITE = 263
    QERR_PRINT_NUM_WRITTEN_IS_NULL = 264
    QERR_STREAM_FORMAT_NOT_DEDUCED = 265
    QERR_MISMATCHED_STREAM_FORMAT = 266
    QERR_DIRECTX_NOT_INSTALLED = 267
    QERR_NO_GAME_CONTROLLERS_ATTACHED = 268
    QERR_CANNOT_ACCESS_GAME_CONTROLLER = 269
    QERR_CANNOT_SET_GAME_CONTROLLER_FORMAT = 270
    QERR_CANNOT_POLL_GAME_CONTROLLER = 271
    QERR_CANNOT_GET_GAME_CONTROLLER_STATE = 272
    QERR_CANNOT_GET_MOUSE_STATE = 273
    QERR_ETHERCAT_INVALID_BYTE_SIZE = 274
    QERR_INVALID_ETHERCAT_SOURCE = 275
    QERR_INVALID_ETHERCAT_INITIALIZATION_DATA = 276
    QERR_INVALID_ETHERCAT_DATA_LENGTH = 277
    QERR_CANNOT_GET_BUFFER_SIZE = 278
    QERR_CANNOT_GET_PACKET_SIZE = 279
    QERR_DATAGRAM_TOO_LARGE = 280
    QERR_NO_DESIGNATED_PEER = 281
    QERR_CANNOT_INDICATE_CLOSURE = 282
    QERR_INVALID_PEER_OPTION = 283
    QERR_CANNOT_BROADCAST = 284
    QERR_ETHERCAT_VALIDATE_DATA_WRONG_LENGTH = 285
    QERR_INVALID_ETHERCAT_VALIDATION_DATA = 286
    QERR_ETHERCAT_VALIDATE_MASK_WRONG_LENGTH = 287
    QERR_INVALID_ETHERCAT_VALIDATION_MASK = 288
    QERR_INVALID_ETHERCAT_TIMEOUT = 289
    QERR_INVALID_ETHERCAT_BEFORE_SLAVE = 290
    QERR_INVALID_ETHERCAT_TRANSITION = 291
    QERR_INVALID_ETHERCAT_REQUIRES_FIELD = 292
    QERR_INVALID_ETHERCAT_COMMAND = 293
    QERR_INVALID_ETHERCAT_LOGICAL_ADDRESS = 294
    QERR_INVALID_ETHERCAT_ADDRESS_PAGE = 295
    QERR_INVALID_ETHERCAT_ADDRESS_OFFSET = 296
    QERR_INVALID_ETHERCAT_COUNT = 297
    QERR_INVALID_ETHERCAT_RETRIES = 298
    QERR_INVALID_ETHERCAT_START_ADDRESS = 299
    QERR_INVALID_ETHERCAT_MAILBOX_SIZE = 300
    QERR_INVALID_ETHERCAT_SLAVE_ADDRESS = 301
    QERR_INVALID_ETHERCAT_STATE = 302
    QERR_INVALID_ETHERCAT_INPUT_OFFSET = 303
    QERR_INVALID_ETHERCAT_OUTPUT_OFFSET = 304
    QERR_OUT_OF_BAND_DATA_NOT_SUPPORTED = 305
    QERR_NO_CORRESPONDING_INTERNET_ADDRESS = 306
    QERR_CANNOT_SET_DESCRIPTOR_FLAGS = 307
    QERR_NO_ACCESS_TO_SHARED_MEMORY = 308
    QERR_SEMAPHORE_NOT_FOUND = 309
    QERR_SEMAPHORE_ALREADY_EXISTS = 310
    QERR_NO_CORRESPONDING_NETWORK_CARD = 311
    QERR_PATH_IN_URI = 312
    QERR_UNSUPPORTED_BAUD_RATE = 313
    QERR_ETHERCAT_MASTER_ALREADY_RUNNING = 314
    QERR_MISSING_CLOCK_MODES = 315
    QERR_MISSING_ENCODER_COUNTS = 316
    QERR_MISSING_PWM_MODES = 317
    QERR_MISSING_PWM_FREQUENCIES = 318
    QERR_MISSING_PWM_DUTY_CYCLES = 319
    QERR_INVALID_NUMBER_OF_SAMPLES_IN_BUFFER = 320
    QERR_INVALID_NUMBER_OF_SAMPLES = 321
    QERR_ETHERCAT_DATAGRAM_TOO_LARGE = 322
    QERR_NO_MORE_ETHERCAT_PACKETS = 323
    QERR_INVALID_ETHERCAT_CYCLIC_COMMAND = 324
    QERR_AUTOPILOT_ARGUMENT_IS_NULL = 325
    QERR_AUTOPILOT_TYPE_ARGUMENT_IS_NULL = 326
    QERR_INVALID_AUTOPILOT_HANDLE = 327
    QERR_URI_HOSTNAME_TOO_LONG = 328
    QERR_URI_SCHEME_TOO_LONG = 329
    QERR_INVALID_CHANNEL = 330
    QERR_ARCNET_NODE_ID_OUT_OF_BOUNDS = 331
    QERR_ARCNET_CANNOT_OPEN = 332
    QERR_ARCNET_TARGET_NODE_DNE = 333
    QERR_ARCNET_EXCESSIVE_NAKS = 334
    QERR_ETHERCAT_PACKET_LOST = 335
    QERR_ETHERCAT_TELEGRAM_LOST = 336
    QERR_INVALID_ETHERCAT_FRAME_LENGTH = 337
    QERR_ETHERCAT_COMMAND_TIMED_OUT = 338
    QERR_INVALID_ETHERCAT_TELEGRAM_LENGTH = 339
    QERR_INVALID_ETHERCAT_MASTER_STATE = 340
    QERR_INVALID_THREAD = 341
    QERR_CANNOT_INITIALIZE_PA10 = 342
    QERR_CANNOT_CLOSE_PA10 = 343
    QERR_PGR_CANNOT_INITIALIZE_CAMERA = 344
    QERR_PGR_CANNOT_GRAB_IMAGE = 345
    QERR_PGR_GRAB_IMAGE_TIMEOUT = 346
    QERR_PGR_CANNOT_CLOSE = 347
    QERR_PGR_INVALID_CUSTOM_IMAGE_SIZE = 348
    QERR_PGR_INVALID_IMAGE_DIMS = 349
    QERR_IMAGE_CANNOT_CONVERT = 350
    QERR_MISSING_ETHERCAT_COMMAND = 351
    QERR_MISSING_ETHERCAT_LOGICAL_ADDRESS = 352
    QERR_MISSING_ETHERCAT_ADDRESS_PAGE = 353
    QERR_MISSING_ETHERCAT_ADDRESS_OFFSET = 354
    QERR_MISSING_ETHERCAT_COMMENT = 355
    QERR_MISSING_ETHERCAT_DATA = 356
    QERR_MISSING_ETHERCAT_VALIDATE_DATA = 357
    QERR_MISSING_ETHERCAT_TRANSITION = 358
    QERR_MISSING_ETHERCAT_RETRIES = 359
    QERR_MISSING_ETHERCAT_SLAVE_INFO = 360
    QERR_MISSING_ETHERCAT_SLAVE_COMMANDS = 361
    QERR_MISSING_ETHERCAT_SLAVE_COMMAND = 362
    QERR_MISSING_ETHERCAT_SLAVE_NAME = 363
    QERR_MISSING_ETHERCAT_CYCLIC_STATE = 364
    QERR_MISSING_ETHERCAT_MASTER_INFO = 365
    QERR_MISSING_ETHERCAT_MASTER_SOURCE = 366
    QERR_MISSING_ETHERCAT_MASTER_COMMANDS = 367
    QERR_MISSING_ETHERCAT_MASTER_COMMAND = 368
    QERR_MISSING_ETHERCAT_MAILBOX_START_ADDRESS = 369
    QERR_MISSING_ETHERCAT_MAILBOX_COUNT = 370
    QERR_MISSING_ETHERCAT_CONFIGURATION = 371
    QERR_MISSING_ETHERCAT_SLAVE = 372
    QERR_MISSING_ETHERCAT_MASTER = 373
    QERR_MISSING_ETHERCAT_PROCESS_IMAGE = 374
    QERR_MISSING_ETHERCAT_PROCESS_INPUTS = 375
    QERR_MISSING_ETHERCAT_PROCESS_INPUTS_SIZE = 376
    QERR_MISSING_ETHERCAT_PROCESS_OUTPUTS = 377
    QERR_MISSING_ETHERCAT_PROCESS_OUTPUTS_SIZE = 378
    QERR_NO_DYNAMIC_RECONFIGURATION_LICENSE = 379
    QERR_NO_COMMUNICATIONS_LICENSE = 380
    QERR_NO_ETHERCAT_LICENSE = 381
    QERR_CANNOT_OPEN_LICENSE_FILE = 382
    QERR_INVALID_LICENSE_FILE = 383
    QERR_CANNOT_CREATE_PREFERENCES_NODE = 384
    QERR_MISSING_ANALOG_MINIMUMS = 385
    QERR_MISSING_ANALOG_MAXIMUMS = 386
    QERR_CANNOT_READ_SCHUNK_GRIPPER = 387
    QERR_SCHUNK_CANNOT_INIT = 388
    QERR_SCHUNK_CANNOT_HALT = 389
    QERR_SCHUNK_CANNOT_CLOSE = 390
    QERR_SCHUNK_CANNOT_MOVE_POS = 391
    QERR_SCHUNK_CANNOT_MOVE_VEL = 392
    QERR_SCHUNK_CANNOT_MOVE_CUR = 393
    QERR_SCHUNK_CANNOT_HOME = 394
    QERR_SCHUNK_INVALID_CONTROL_MODE = 395
    QERR_SCHUNK_EXT_MODE_NOT_CONNECTED = 396
    QERR_INVALID_ANALOG_INPUT_RANGE = 397
    QERR_INVALID_ANALOG_OUTPUT_RANGE = 398
    QERR_CARD_IDENTIFIER_ARGUMENT_IS_NULL = 399
    QERR_BOARD_IDENTIFIER_ARGUMENT_IS_NULL = 400
    QERR_INVALID_BOARD_IDENTIFIER = 401
    QERR_INVALID_DEVICE_HANDLE = 402
    QERR_CANNOT_OPEN_DRIVER_DIRECTORY = 403
    QERR_WIIMOTE_WRITE_REPORT_FAILED = 404
    QERR_WIIMOTE_CANNOT_CALIBRATE = 405
    QERR_WIIMOTE_CANNOT_OPEN = 406
    QERR_WIIMOTE_READ_FAILED = 407
    QERR_WIIMOTE_NOT_FOUND = 408
    QERR_INVALID_WIIMOTE_DEVICE_HANDLE = 409
    QERR_DAQMX_CANNOT_CLEAR_TASK = 410
    QERR_DAQMX_CANNOT_CREATE_TASK = 411
    QERR_DAQMX_CANNOT_ATTACH_ADC_TO_TASK = 412
    QERR_DAQMX_CANNOT_START_TASK = 413
    QERR_DAQMX_ERROR_SAMPLING_ADC = 414
    QERR_DAQMX_ERROR_RESETTING_DEVICE = 415
    QERR_DAQMX_CANNOT_ATTACH_DAC_TO_TASK = 416
    QERR_DAQMX_ERROR_WRITING_DAC = 417
    QERR_ANALOG_OUTPUT_RANGE_DIFF = 418
    QERR_ANALOG_INPUT_RANGE_DIFF = 419
    QERR_DAQMX_CANNOT_SET_HARDWARE_CLOCK_RATE = 420
    QERR_DAQMX_CLOCK_ERROR_WAITING_FOR_SAMPLE = 421
    QERR_DAQMX_CANNOT_ATTACH_ENCODER_TO_TASK = 422
    QERR_DAQMX_ERROR_SAMPLING_ENCODER = 423
    QERR_DAQMX_ERROR_CHANGING_ENC_DIR_SRC = 424
    QERR_DAQMX_CANNOT_ATTACH_DIGITAL_TO_TASK = 425
    QERR_DAQMX_ERROR_SAMPLING_DIGITAL = 426
    QERR_DAQMX_ERROR_WRITING_DIGITAL = 427
    QERR_JR3PCI_CANNOT_INIT = 428
    QERR_BUFFER_IS_NULL = 429
    QERR_DIGITAL_OUTPUT_LOCKED = 430
    QERR_ANALOG_OUTPUT_LOCKED = 431
    QERR_PWM_OUTPUT_LOCKED = 432
    QERR_MISSING_URI_OPTION_NAME = 433
    QERR_ENCODER_QUADRATURE_MODE_NOT_SUPPORTED = 434
    QERR_MISSING_ENCODER_QUADRATURE_MODES = 435
    QERR_INVALID_ENCODER_QUADRATURE_MODE = 436
    QERR_MISSING_ENCODER_FILTER_FREQUENCIES = 437
    QERR_INVALID_ENCODER_FILTER_FREQUENCY = 438
    QERR_HIL_READ_NOT_SUPPORTED = 439
    QERR_HIL_READ_ANALOG_NOT_SUPPORTED = 440
    QERR_HIL_READ_ANALOG_BUFFER_NOT_SUPPORTED = 441
    QERR_HIL_READ_ANALOG_CODES_NOT_SUPPORTED = 442
    QERR_HIL_READ_ANALOG_WRITE_ANALOG_NOT_SUPPORTED = 443
    QERR_HIL_READ_ANALOG_WRITE_ANALOG_BUFFER_NOT_SUPPORTED = 444
    QERR_HIL_READ_BUFFER_NOT_SUPPORTED = 445
    QERR_HIL_READ_DIGITAL_NOT_SUPPORTED = 446
    QERR_HIL_READ_DIGITAL_BUFFER_NOT_SUPPORTED = 447
    QERR_HIL_READ_DIGITAL_WRITE_DIGITAL_NOT_SUPPORTED = 448
    QERR_HIL_READ_DIGITAL_WRITE_DIGITAL_BUFFER_NOT_SUPPORTED = 449
    QERR_HIL_READ_ENCODER_NOT_SUPPORTED = 450
    QERR_HIL_READ_ENCODER_BUFFER_NOT_SUPPORTED = 451
    QERR_HIL_READ_ENCODER_WRITE_PWM_NOT_SUPPORTED = 452
    QERR_HIL_READ_ENCODER_WRITE_PWM_BUFFER_NOT_SUPPORTED = 453
    QERR_HIL_READ_OTHER_NOT_SUPPORTED = 454
    QERR_HIL_READ_OTHER_BUFFER_NOT_SUPPORTED = 455
    QERR_HIL_READ_OTHER_WRITE_OTHER_NOT_SUPPORTED = 456
    QERR_HIL_READ_OTHER_WRITE_OTHER_BUFFER_NOT_SUPPORTED = 457
    QERR_HIL_READ_WRITE_NOT_SUPPORTED = 458
    QERR_HIL_READ_WRITE_BUFFER_NOT_SUPPORTED = 459
    QERR_HIL_SET_ANALOG_INPUT_RANGES_NOT_SUPPORTED = 460
    QERR_HIL_SET_ANALOG_OUTPUT_RANGES_NOT_SUPPORTED = 461
    QERR_HIL_SET_CARD_SPECIFIC_OPTIONS_NOT_SUPPORTED = 462
    QERR_HIL_SET_CLOCK_MODE_NOT_SUPPORTED = 463
    QERR_HIL_SET_DIGITAL_DIRECTIONS_NOT_SUPPORTED = 464
    QERR_HIL_SET_ENCODER_COUNTS_NOT_SUPPORTED = 465
    QERR_HIL_SET_ENCODER_FILTER_FREQUENCY_NOT_SUPPORTED = 466
    QERR_HIL_SET_ENCODER_QUADRATURE_MODE_NOT_SUPPORTED = 467
    QERR_HIL_SET_PWM_DUTY_CYCLE_NOT_SUPPORTED = 468
    QERR_HIL_SET_PWM_FREQUENCY_NOT_SUPPORTED = 469
    QERR_HIL_SET_PWM_MODE_NOT_SUPPORTED = 470
    QERR_HIL_TASK_CREATE_ANALOG_READER_NOT_SUPPORTED = 471
    QERR_HIL_TASK_CREATE_ANALOG_READER_ANALOG_WRITER_NOT_SUPPORTED = 472
    QERR_HIL_TASK_CREATE_DIGITAL_READER_NOT_SUPPORTED = 473
    QERR_HIL_TASK_CREATE_DIGITAL_READER_DIGITAL_WRITER_NOT_SUPPORTED = 474
    QERR_HIL_TASK_CREATE_DIGITAL_WRITER_NOT_SUPPORTED = 475
    QERR_HIL_TASK_CREATE_ENCODER_READER_NOT_SUPPORTED = 476
    QERR_HIL_TASK_CREATE_ENCODER_READER_PWM_WRITER_NOT_SUPPORTED = 477
    QERR_HIL_TASK_CREATE_OTHER_READER_NOT_SUPPORTED = 478
    QERR_HIL_TASK_CREATE_OTHER_READER_OTHER_WRITER_NOT_SUPPORTED = 479
    QERR_HIL_TASK_CREATE_OTHER_WRITER_NOT_SUPPORTED = 480
    QERR_HIL_TASK_CREATE_PWM_WRITER_NOT_SUPPORTED = 481
    QERR_HIL_TASK_CREATE_READER_NOT_SUPPORTED = 482
    QERR_HIL_TASK_CREATE_READER_WRITER_NOT_SUPPORTED = 483
    QERR_HIL_TASK_CREATE_WRITER_NOT_SUPPORTED = 484
    QERR_HIL_TASK_DELETE_NOT_SUPPORTED = 485
    QERR_HIL_TASK_FLUSH_NOT_SUPPORTED = 486
    QERR_HIL_TASK_READ_NOT_SUPPORTED = 487
    QERR_HIL_TASK_READ_ANALOG_NOT_SUPPORTED = 488
    QERR_HIL_TASK_READ_ANALOG_WRITE_ANALOG_NOT_SUPPORTED = 489
    QERR_HIL_TASK_READ_DIGITAL_NOT_SUPPORTED = 490
    QERR_HIL_TASK_READ_DIGITAL_WRITE_DIGITAL_NOT_SUPPORTED = 491
    QERR_HIL_TASK_READ_ENCODER_NOT_SUPPORTED = 492
    QERR_HIL_TASK_READ_ENCODER_WRITE_PWM_NOT_SUPPORTED = 493
    QERR_HIL_TASK_READ_OTHER_NOT_SUPPORTED = 494
    QERR_HIL_TASK_READ_OTHER_WRITE_OTHER_NOT_SUPPORTED = 495
    QERR_HIL_TASK_READ_WRITE_NOT_SUPPORTED = 496
    QERR_HIL_TASK_START_NOT_SUPPORTED = 497
    QERR_HIL_TASK_STOP_NOT_SUPPORTED = 498
    QERR_HIL_TASK_WRITE_NOT_SUPPORTED = 499
    QERR_HIL_TASK_WRITE_ANALOG_NOT_SUPPORTED = 500
    QERR_HIL_TASK_WRITE_DIGITAL_NOT_SUPPORTED = 501
    QERR_HIL_TASK_WRITE_OTHER_NOT_SUPPORTED = 502
    QERR_HIL_TASK_WRITE_PWM_NOT_SUPPORTED = 503
    QERR_HIL_WRITE_NOT_SUPPORTED = 504
    QERR_HIL_WRITE_ANALOG_NOT_SUPPORTED = 505
    QERR_HIL_WRITE_ANALOG_BUFFER_NOT_SUPPORTED = 506
    QERR_HIL_WRITE_ANALOG_CODES_NOT_SUPPORTED = 507
    QERR_HIL_WRITE_BUFFER_NOT_SUPPORTED = 508
    QERR_HIL_WRITE_DIGITAL_NOT_SUPPORTED = 509
    QERR_HIL_WRITE_DIGITAL_BUFFER_NOT_SUPPORTED = 510
    QERR_HIL_WRITE_OTHER_NOT_SUPPORTED = 511
    QERR_HIL_WRITE_OTHER_BUFFER_NOT_SUPPORTED = 512
    QERR_HIL_WRITE_PWM_NOT_SUPPORTED = 513
    QERR_HIL_WRITE_PWM_BUFFER_NOT_SUPPORTED = 514
    QERR_Q3_CONTROLPAQ_FW_CANNOT_OPEN_BOARD = 515
    QERR_CANNOT_CREATE_GAME_CONTROLLER_WINDOW = 516
    QERR_CANNOT_SET_GAME_CONTROLLER_COOPERATIVE_LEVEL = 517
    QERR_CONNECTION_NOT_BOUND = 518
    QERR_NO_SIMULINK_DEVELOPMENT_LICENSE = 519
    QERR_INVALID_CIRCULAR_BUFFER = 520
    QERR_READ_TOO_LONG = 521
    QERR_WRITE_TOO_LONG = 522
    QERR_TOO_MANY_FORCE_FEEDBACK_EFFECTS = 523
    QERR_INVALID_FORCE_FEEDBACK_AXIS = 524
    QERR_CANNOT_CREATE_FORCE_FEEDBACK_EFFECT = 525
    QERR_CANNOT_START_FORCE_FEEDBACK_EFFECT = 526
    QERR_CANNOT_STOP_FORCE_FEEDBACK_EFFECT = 527
    QERR_CANNOT_SET_FORCE_FEEDBACK_EFFECT_PARAMETERS = 528
    QERR_INVALID_FORCE_FEEDBACK_EFFECT = 529
    QERR_INVALID_GAME_CONTROLLER = 530
    QERR_GAME_CONTROLLER_NOT_FOUND = 531
    QERR_CANNOT_START_TARGET_MANAGER = 532
    QERR_CANNOT_STOP_TARGET_MANAGER = 533
    QERR_PEER_IGNORING_SHUTDOWN = 534
    QERR_INVALID_PERIODIC_EFFECT_TYPE = 535
    QERR_INVALID_CONDITION_EFFECT_TYPE = 536
    QERR_TOO_MANY_GAME_CONTROLLER_AXES = 537
    QERR_INVALID_NUMBER_OF_CONDITIONS = 538
    QERR_INCOMPLETE_READ = 539
    QERR_EXCLUSIVE_ACCESS_ALREADY_GRANTED = 540
    QERR_EXCLUSIVE_ACCESS_NOT_GRANTED = 541
    QERR_CARD_LOCATION_IS_NULL = 542
    QERR_HIL_ACQUIRE_EXCLUSIVE_ACCESS_NOT_SUPPORTED = 543
    QERR_HIL_RELEASE_EXCLUSIVE_ACCESS_NOT_SUPPORTED = 544
    QERR_DRIVER_MISSING_GET_OR_RELEASE_ACCESS = 545
    QERR_CIRCULAR_BUFFER_NOT_FOUND = 546
    QERR_INCOMPATIBLE_PIPE = 547
    QERR_SENSORAY_TOO_MANY_BOARDS = 548
    QERR_SENSORAY_ILLEGAL_PARAM = 549
    QERR_SENSORAY_EEPROM_ERROR = 550
    QERR_SENSORAY_UNSPECIFIED_ERROR = 551
    QERR_KERNEL_CANNOT_REGISTER_BOARD = 552
    QERR_DMA_BUFFER_LOCK = 553
    QERR_CANNOT_START_INTERRUPT_THREAD = 554
    QERR_DAC_COMM_TIMEOUT = 555
    QERR_COUNTER_RESOURCE_CONFLICT = 556
    QERR_STREAM_NOT_CONNECTED = 557
    QERR_LIBRARY_LOAD_ERROR = 558
    QERR_PHANTOM_OMNI_CANNOT_OPEN_BOARD = 559
    QERR_JR3PCI_CANNOT_SET_FULL_SCALES = 560
    QERR_ALTIA_ARGUMENT_IS_NULL = 561
    QERR_INVALID_ALTIA = 562
    QERR_ALTIA_INPUT_IS_NULL = 563
    QERR_ALTIA_OUTPUT_IS_NULL = 564
    QERR_INVALID_ALTIA_EVENT_NAME = 565
    QERR_INVALID_ALTIA_INPUT = 566
    QERR_INVALID_ALTIA_OUTPUT = 567
    QERR_BEEP_ARGUMENT_IS_NULL = 568
    QERR_INVALID_BEEP = 569
    QERR_BEEP_FAILED = 570
    QERR_BEEP_FREQUENCY_OUT_OF_RANGE = 571
    QERR_SCAN_VALUE_IS_NULL = 572
    QERR_CARD_SPECIFIC_OPTION_NOT_RECOGNIZED = 573
    QERR_CARD_SPECIFIC_OPTION_VALUE_NOT_RECOGNIZED = 574
    QERR_CARD_SPECIFIC_OPTION_NOT_SUPPORTED = 575
    QERR_INVALID_ROOMBA = 576
    QERR_INVALID_ROOMBA_SENSOR_ID = 577
    QERR_VISION_CAMERA_NOT_FOUND = 578
    QERR_INVALID_IPLIMAGE = 579
    QERR_SAVE_IMAGE = 580
    QERR_INIT_V4L2_DEVICE = 581
    QERR_GRAB_V4L2_IMAGE = 582
    QERR_NO_RPC_SERVER_FOR_BEEP = 583
    QERR_MISMATCHED_CHARACTER = 584
    QERR_EMPTY_SCAN = 585
    QERR_URI_MISSING_HOST = 586
    QERR_PATH_IN_PIPE_URI = 587
    QERR_HOST_IN_PIPE_URI = 588
    QERR_HIQ_UNKNOWN_REPORT_TYPE = 589
    QERR_HIQ_RECEIVE_BLOCKED = 590
    QERR_HIL_WATCHDOG_SET_ANALOG_EXPIRATION_STATE_NOT_SUPPORTED = 591
    QERR_HIL_WATCHDOG_SET_DIGITAL_EXPIRATION_STATE_NOT_SUPPORTED = 592
    QERR_HIL_WATCHDOG_SET_PWM_EXPIRATION_STATE_NOT_SUPPORTED = 593
    QERR_HIL_WATCHDOG_SET_OTHER_EXPIRATION_STATE_NOT_SUPPORTED = 594
    QERR_HIL_WATCHDOG_START = 595
    QERR_HIL_WATCHDOG_STOP = 596
    QERR_HIL_WATCHDOG_RELOAD = 597
    QERR_HIL_WATCHDOG_IS_EXPIRED = 598
    QERR_HIL_WATCHDOG_CLEAR = 599
    QERR_HIL_INVALID_DIGITAL_STATE = 600
    QERR_ANALOG_EXPIRATION_STATE_NOT_ZERO = 601
    QERR_DIGITAL_EXPIRATION_STATE_NOT_TRISTATE = 602
    QERR_CLOCK_NOT_WATCHDOG = 603
    QERR_ANALOG_EXPIRATIONS_NOT_CONFIGURED = 604
    QERR_DIGITAL_EXPIRATIONS_NOT_CONFIGURED = 605
    QERR_CLOCK_PERIOD_TOO_HIGH = 606
    QERR_CLOCK_PERIOD_TOO_LOW = 607
    QERR_HIL_TASK_CREATE_ANALOG_WRITER_NOT_SUPPORTED = 608
    QERR_FALCON_FAILED_TO_INITIALIZE = 609
    QERR_FALCON_COULD_NOT_OPEN_DEVICE = 610
    QERR_FALCON_COULD_NOT_START_DEVICE = 611
    QERR_FALCON_COULD_NOT_CREATE_CALLBACK = 612
    QERR_FALCON_COULD_NOT_MAKE_CURRENT = 613
    QERR_Q8_SERIES_EXPIRATIONS_NOT_CONFIGURED = 614
    QERR_CONNECTION_ABORTED = 615
    QERR_INVALID_ROOMBA_SONG_NUMBER = 616
    QERR_INVALID_ROOMBA_SONG_LENGTH = 617
    QERR_INVALID_ROOMBA_NOTE_NUMBER = 618
    QERR_INVALID_ROOMBA_DIGITAL_OUTPUT = 619
    QERR_INVALID_ROOMBA_EVENT_NO = 620
    QERR_INVALID_ROOMBA_MODE = 621
    QERR_INVALID_ROOMBA_DEMO = 622
    QERR_INVALID_ROOMBA_LED_BITS = 623
    QERR_INVALID_ROOMBA_SCRIPT_LENGTH = 624
    QERR_PREFERENCES_VALUE_CONTAINS_ENVIRONMENT_VARIABLES = 625
    QERR_INVALID_TYPE_OF_PREFERENCES_VALUE = 626
    QERR_INVALID_ROOMBA_STREAM_STATE = 627
    QERR_INVALID_ROOMBA_DRIVER_BITS = 628
    QERR_INVALID_ROOMBA_DUTY_CYCLE = 629
    QERR_INVALID_ROOMBA_PACKET_NUMBER = 630
    QERR_INVALID_ROOMBA_STREAM_HEADER = 631
    QERR_CORRUPTED_ROOMBA_STREAM = 632
    QERR_INVALID_ROOMBA_STREAM_SIZE = 633
    QERR_INVALID_IMAGE_DIMENSION = 634
    QERR_INVALID_SERACCEL = 635
    QERR_SERACCEL_COULD_NOT_OPEN_DEVICE = 636
    QERR_SERACCEL_COULD_NOT_START_DEVICE = 637
    QERR_SERACCEL_COULD_NOT_READ_DEVICE = 638
    QERR_NO_ALTIA_LICENSE = 639
    QERR_NO_IROBOT_ROOMBA_LICENSE = 640
    QERR_NO_JR3_FORCE_TORQUE_LICENSE = 641
    QERR_NO_MITSUBISHI_PA10_LICENSE = 642
    QERR_NO_NINTENDO_WIIMOTE_LICENSE = 643
    QERR_NO_NOVINT_FALCON_LICENSE = 644
    QERR_NO_POINTGREY_CAMERAS_LICENSE = 645
    QERR_NO_SCHUNK_GRIPPER_LICENSE = 646
    QERR_NO_SENSABLE_OMNI_LICENSE = 647
    QERR_INVALID_IMAGE_NAME = 648
    QERR_NO_INTERNAL_USE_LICENSE = 649
    QERR_DRAGANFLY_X6_CRC_FAILED = 650
    QERR_CANPCI_NOT_FOUND = 651
    QERR_CANPCI_INIT_FAILED = 652
    QERR_CANPCI_INVALID_PARAMETERS = 653
    QERR_CANPCI_SEND_MESSAGE_FAILED = 654
    QERR_CANPCI_GET_MESSAGE_FAILED = 655
    QERR_CANPCI_INVALID_CHANNEL = 656
    QERR_CANPCI_START_FAILED = 657
    QERR_INVALID_UBLOX = 658
    QERR_NO_UBLOX_MSG = 659
    QERR_INVALID_UBLOX_MSG = 660
    QERR_INVALID_UBLOX_IDENTIFIERS = 661
    QERR_INVALID_UBLOX_CHECKSUM = 662
    QERR_INVALID_NMEA_MSG = 663
    QERR_INVALID_NMEA_CHECKSUM = 664
    QERR_INVALID_UBLOX_DATA = 665
    QERR_UNSUPPORTED_GPS_DATA_FIELD = 666
    QERR_ROBOSTIX_INVALID_PROGRAM_VERSION = 667
    QERR_SPI_TRANSMIT = 668
    QERR_SPI_RECEIVE = 669
    QERR_NO_SUCH_DEVICE = 670
    QERR_INTIME_NOT_RUNNING = 671
    QERR_CANNOT_CONNECT_TO_LICENSE_MANAGER = 672
    QERR_MISSING_PROPERTIES = 673
    QERR_MISSING_PROPERTIES_BUFFER = 674
    QERR_HIL_GET_INTEGER_PROPERTY_NOT_SUPPORTED = 675
    QERR_HIL_GET_DOUBLE_PROPERTY_NOT_SUPPORTED = 676
    QERR_HIL_GET_STRING_PROPERTY_NOT_SUPPORTED = 677
    QERR_HIL_SET_INTEGER_PROPERTY_NOT_SUPPORTED = 678
    QERR_HIL_SET_DOUBLE_PROPERTY_NOT_SUPPORTED = 679
    QERR_HIL_SET_STRING_PROPERTY_NOT_SUPPORTED = 680
    QERR_PROPERTY_NOT_RECOGNIZED = 681
    QERR_GUMSTIX_WATCHDOG_CLOCK_PERIOD_TOO_HIGH = 682
    QERR_GUMSTIX_WATCHDOG_CLOCK_PERIOD_TOO_LOW = 683
    QERR_DIGITAL_INPUTS_NOT_INITIALIZED = 684
    QERR_DIGITAL_OUTPUTS_NOT_INITIALIZED = 685
    QERR_FILTER_PROTOCOLS_REQUIRE_URI = 686
    QERR_CONFLICTING_COUNTER_MODES = 687
    QERR_DAQMX_ERROR_CHANGING_PWM_OUT_TERM = 688
    QERR_DAQMX_ERROR_WRITING_PWM = 689
    QERR_DAQMX_CANNOT_ATTACH_PWM_TO_TASK = 690
    QERR_DAQMX_ERROR_SETTING_IMPLICIT_TIMING = 691
    QERR_DAQMX_CANNOT_GET_PWM_CHANNEL_NAME = 692
    QERR_NI_DUTY_CYCLE_OUT_OF_RANGE = 693
    QERR_NI_FREQUENCY_OUT_OF_RANGE = 694
    QERR_DAQMX_ERROR_CHANGING_TIMEBASE_RATE = 695
    QERR_INVALID_DSR_CONTROL = 696
    QERR_OPTITRACK_RIGID_BODY_INIT_ERROR = 697
    QERR_OPTITRACK_POINT_CLOUD_INIT_ERROR = 698
    QERR_OPTITRACK_RIGID_BODY_ID_INVALID = 699
    QERR_OPTITRACK_ERROR_STARTING_CAMERAS = 700
    QERR_OPTITRACK_ERROR_STOPPING_CAMERAS = 701
    QERR_OPTITRACK_INVALID_CALIBRATION_FILE = 702
    QERR_OPTITRACK_INVALID_RIGID_BODY_FILE = 703
    QERR_OPTITRACK_TOO_MANY_RIGID_BODIES = 704
    QERR_VISION_INVALID_PARAMETER = 705
    QERR_VISION_INVALID_NO_OF_CHANNELS = 706
    QERR_NO_CANCELLATION_HANDLER = 707
    QERR_VISION_INVALID_INPUT = 708
    QERR_PORT_UNREACHABLE = 709
    QERR_CANNOT_SET_PORT_UNREACHABLE = 710
    QERR_MUST_BE_ADMINISTRATOR = 711
    QERR_MISMATCHED_ENCODER_FILTER_FREQUENCY = 712
    QERR_MISMATCHED_CLOCK_FREQUENCY = 713
    QERR_UNABLE_TO_OPEN_DIALOG = 714
    QERR_HIL_SET_DIGITAL_OUTPUT_CONFIGURATION_NOT_SUPPORTED = 715
    QERR_HIL_SET_PWM_CONFIGURATION_NOT_SUPPORTED = 716
    QERR_HIL_SET_PWM_DEADBAND_NOT_SUPPORTED = 717
    QERR_MISSING_DIGITAL_CONFIGURATIONS = 718
    QERR_MISSING_PWM_CONFIGURATIONS = 719
    QERR_MISSING_PWM_ALIGNMENTS = 720
    QERR_MISSING_PWM_POLARITIES = 721
    QERR_MISSING_PWM_DEADBANDS = 722
    QERR_INVALID_PWM_CONFIGURATION = 723
    QERR_INVALID_PWM_ALIGNMENT = 724
    QERR_INVALID_PWM_POLARITY = 725
    QERR_PWM_CONFIGURATION_NOT_SUPPORTED = 726
    QERR_INVALID_PWM_DEADBAND = 727
    QERR_BIPOLAR_PWM_ON_EVEN_CHANNELS_ONLY = 728
    QERR_GPS_READ_FAILED = 729
    QERR_PWM_ALIGNMENT_NOT_SUPPORTED = 730
    QERR_PWM_POLARITY_NOT_SUPPORTED = 731
    QERR_PWM_DEADBAND_NOT_SUPPORTED = 732
    QERR_INVALID_CHANNEL_ORDER_FOR_BIPOLAR_PWM = 733
    QERR_SUM_OF_PWM_DEADBANDS_EXCEEDS_PERIOD = 734
    QERR_PWM_MODES_NOT_ONE_SHOT = 735
    QERR_PWM_EXPIRATIONS_NOT_CONFIGURED = 736
    QERR_PHANTOM_CANNOT_INITIALIZE = 737
    QERR_PHANTOM_SCHEDULER_ERROR = 738
    QERR_PHANTOM_SCHEDULER_RATE_ERROR = 739
    QERR_PHANTOM_READ_FAILED = 740
    QERR_PHANTOM_WRITE_FAILED = 741
    QERR_PHANTOM_CANNOT_CLOSE = 742
    QERR_NO_PHANTOM_OMNI_LICENSE = 743
    QERR_NO_PHANTOM_DESKTOP_LICENSE = 744
    QERR_NO_PHANTOM_PREMIUM_LICENSE = 745
    QERR_NO_PHANTOM_PREMIUM_6DOF_LICENSE = 746
    QERR_VAL_QBOT_OPEN_FAILED = 747
    QERR_INVALID_KR5_SIXX_R850 = 748
    QERR_KR5_SIXX_R850_COULD_NOT_OPEN_DEVICE = 749
    QERR_NO_KUKA_KR5_SIXX_R850_LICENSE = 750
    QERR_VISION_INVALID_IMAGE_SIZE = 751
    QERR_INVALID_HYMOTION_11000 = 752
    QERR_HYMOTION_11000_COULD_NOT_OPEN_DEVICE = 753
    QERR_NO_REXROTH_HYMOTION_11000_LICENSE = 754
    QERR_INVALID_SHMEM_SCOPE = 755
    QERR_NO_NATURALPOINT_OPTITRACK_LICENSE = 756
    QERR_NO_VISUALIZATION_LICENSE = 757
    QERR_NO_GPS_LICENSE = 758
    QERR_INVALID_CIGI_HOST = 759
    QERR_CIGI_HOST_COULD_NOT_OPEN_DEVICE = 760
    QERR_INVALID_DEFLATE_MODE = 761
    QERR_STATE_IS_NULL = 762
    QERR_WRONG_NUM_BYTES_POKED = 763
    QERR_WRONG_NUM_BYTES_PEEKED = 764
    QERR_NO_QBOT_LICENSE = 765
    QERR_NO_UAV_LICENSE = 766
    QERR_PHANTOM_LIBRARY_OPEN_FAILED = 767
    QERR_WRONG_NUMBER_OF_INITIAL_VALUES = 768
    QERR_INVALID_NEES = 769
    QERR_NEES_COULD_NOT_INITIALIZE = 770
    QERR_NEES_COULD_NOT_COMMUNICATE = 771
    QERR_NEES_INVALID_DATA_FROM_DAEMON = 772
    QERR_NEES_COULD_NOT_CLOSE = 773
    QERR_TCP_KEEPALIVES_NOT_SUPPORTED = 774
    QERR_CANNOT_SELECT_VARIABLE = 775
    QERR_INVALID_VARIABLE_NAME = 776
    QERR_VARIABLE_ALREADY_SELECTED = 777
    QERR_INVALID_VARIABLE_DATATYPE = 778
    QERR_QERR_CANNOT_SET_VARIABLE = 779
    QERR_VARIABLE_NOT_SELECTED = 780
    QERR_VISUALIZATION_ALREADY_STARTED = 781
    QERR_UNABLE_TO_START_VIEWER = 782
    QERR_INVALID_VISUALIZATION_HANDLE = 783
    QERR_VIEWER_MAY_NOT_HAVE_EXITED = 784
    QERR_PCAN_CANNOT_INITIALIZE = 785
    QERR_WIIMOTION_PLUS_ACTIVATE_FAILED = 786
    QERR_WIIMOTION_PLUS_NOT_DETECTED = 787
    QERR_WIIMOTE_EXT_CONTROLLER_CHECK_FAILED = 788
    QERR_SOFTWARE_ALREADY_INSTALLED = 789
    QERR_NO_SPACE_ON_FILE_SYSTEM = 790
    QERR_FILE_SYSTEM_ERROR = 791
    QERR_INVALID_SETUP_OPERATION = 792
    QERR_PREMATURE_END_OF_FILE = 793
    QERR_PARTITION_NOT_FOUND = 794
    QERR_PARTITIONING_SCHEDULER_NOT_RUNNING = 795
    QERR_JOINING_PARTITION_DENIED = 796
    QERR_DIGITAL_EXPIRATION_STATE_TRISTATE_INVALID = 797
    QERR_INVALID_LICENSE_FOR_BUILDFILE = 798
    QERR_URI_OPTION_NOT_RECOGNIZED = 799
    QERR_HIL_CONFLICTING_DIGITAL_OUTPUTS = 800
    QERR_DENSO_READ_TIMEOUT = 801
    QERR_NO_ROOM_IN_RECEIVE_BUFFER = 802
    QERR_NO_DATA_IN_RECEIVE_BUFFER = 803
    QERR_SPI_WRONG_BYTES_TO_SEND_AND_RECEIVE = 804
    QERR_NO_SUCH_SPI_CHANNEL = 805
    QERR_ONLY_SPI_MASTER_MODE_SUPPORTED = 806
    QERR_ONLY_SPI_MSB_FIRST_SUPPORTED = 807
    QERR_NO_DENSO_LICENSE = 808
    QERR_NO_PTI_VISUALEYEZ_LICENSE = 809
    QERR_PTI_VZSOFT_FAILED_TO_INITIALIZE = 810
    QERR_PTI_VZANALYZER_FAILED_TO_INITIALIZE = 811
    QERR_PWM_MODES_NOT_COMPATIBLE = 812
    QERR_NP_TRACK_IR_OPEN_FAILED = 813
    QERR_CANNOT_SET_POSITION = 814
    QERR_SEMAPHORE_COUNT_EXCEEDED = 815
    QERR_CONNECTION_REFUSED = 816
    QERR_MISSING_INTERRUPT_SOURCES = 817
    QERR_MISSING_INTERRUPT_OCCURRED_BUFFER = 818
    QERR_HIL_POLL_INTERRUPT_NOT_SUPPORTED = 819
    QERR_MONITOR_ARGUMENT_IS_NULL = 820
    QERR_INVALID_MONITOR_HANDLE = 821
    QERR_HIL_MONITOR_CREATE_INTERRUPT_READER_NOT_SUPPORTED = 822
    QERR_HIL_MONITOR_START_NOT_SUPPORTED = 823
    QERR_HIL_MONITOR_STOP_NOT_SUPPORTED = 824
    QERR_HIL_MONITOR_DELETE_NOT_SUPPORTED = 825
    QERR_HIL_MONITOR_READ_INTERRUPT_NOT_SUPPORTED = 826
    QERR_INVALID_INTERRUPT_SOURCE = 827
    QERR_INVALID_INTERRUPT_OPERATION_HANDLE = 828
    QERR_INTERRUPT_OPERATION_ARGUMENT_IS_NULL = 829
    QERR_OPTITRACK_INVALID_LICENSE = 830
    QERR_OPTITRACK_FRAME_UPDATE_FAILED = 831
    QERR_TOO_MANY_INTERRUPT_SOURCES = 832
    QERR_HIL_SET_CLOCK_FREQUENCY_NOT_SUPPORTED = 833
    QERR_VICON_CANNOT_CONNECT = 834
    QERR_VICON_FAILED_TO_SET_STREAM_MODE = 835
    QERR_VICON_ENABLE_UNLABELED_MARKER_DATA_FAILED = 836
    QERR_VICON_ENABLE_MARKER_DATA_FAILED = 837
    QERR_VICON_ENABLE_SEGMENT_DATA_FAILED = 838
    QERR_NO_VICON_LICENSE = 839
    QERR_DAQMX_ONLY_DEVICE_NAMES_SUPPORTED = 840
    QERR_HIQ_BUILD_NUMBER_MISMATCH = 841
    QERR_INVALID_HOST_PACKET = 842
    QERR_DRIVER_TIMED_OUT = 843
    QERR_V4L2_VIDIOC_QBUF_FAILED = 844
    QERR_V4L2_COULD_NOT_CLOSE = 845
    QERR_V4L2_DEVICE_MISSING = 846
    QERR_V4L2_QUERY_FAILED = 847
    QERR_V4L2_NOT_CAP_DEVICE = 848
    QERR_V4L2_NOT_STREAMING_DEVICE = 849
    QERR_V4L2_FORMAT_NOT_VALID = 850
    QERR_V4L2_VIDIOC_REQBUFS_FAILED = 851
    QERR_V4L2_VIDIOC_STREAM_FAILED = 852
    QERR_CASPA_CAPTURE_FAILED = 853
    QERR_CARD_NOT_ACTIVE = 854
    QERR_AUTOPILOT_NOT_ACTIVE = 855
    QERR_PCAN_INVALID_CHANNEL = 856
    QERR_PCAN_SET_MESSAGE_FILTER_FAILED = 857
    QERR_INCOMPATIBLE_TARGET_TYPE = 858
    QERR_PCAN_WRITE_FAILED = 859
    QERR_JACO_INVALID_JOINT = 860
    QERR_JACO_JOINT_INITIALIZATION_FAILED = 861
    QERR_JACO_ACK_NOT_RECEIVED = 862
    QERR_SHARING_VIOLATION = 863
    QERR_JACO_READ_FAILED = 864
    QERR_JACO_JOINT_ADDRESS_INVALID = 865
    QERR_NO_ROBOTICS_LICENSE = 866
    QERR_INVALID_HOST = 867
    QERR_JACO_READ_TIMEOUT = 868
    QERR_JACO_SHUTDOWN = 869
    QERR_PERIPHERAL_NOT_FOUND = 870
    QERR_ENVIRONMENT_VARIABLE_NOT_FOUND = 871
    QERR_VIS_CANNOT_CREATE_FRAME_TIMER = 872
    QERR_VIS_CANNOT_CREATE_VISUALIZATION_WINDOW = 873
    QERR_VIS_CANNOT_REGISTER_WINDOW = 874
    QERR_VIS_CANNOT_GET_WINDOW_DC = 875
    QERR_VIS_FAILED_TO_SWITCH_TO_FULL_SCREEN = 876
    QERR_VIS_CANNOT_FIND_SUITABLE_PIXEL_FORMAT = 877
    QERR_VIS_CANNOT_SET_PIXEL_FORMAT = 878
    QERR_VIS_CANNOT_CREATE_OPENGL_CONTEXT = 879
    QERR_VIS_CANNOT_CREATE_EXTENDED_OPENGL_CONTEXT = 880
    QERR_VIS_CANNOT_MAKE_OPENGL_CONTEXT = 881
    QERR_VIS_CANNOT_MAKE_EXTENDED_OPENGL_CONTEXT = 882
    QERR_VIS_UNABLE_TO_SWAP_GRAPHICS_BUFFERS = 883
    QERR_VIS_TOO_MANY_LIGHTS_IN_SCENE = 884
    QERR_VIS_CANNOT_FIND_VALID_X3D_MESH = 885
    QERR_VIS_CANNOT_FIND_SPECIFIED_SHAPE_IN_X3D_MESH = 886
    QERR_VIS_CANNOT_FIND_SPECIFIED_ATTRIBUTE_IN_X3D_MESH = 887
    QERR_VIS_INVALID_SCENE_FILE_FORMAT = 888
    QERR_VIS_INVALID_MESH_FILE_FORMAT = 889
    QERR_VIS_UNSUPPORTED_TEXTURE_FORMAT = 890
    QERR_VIS_TEXTURE_NOT_POWER_OF_2 = 891
    QERR_VIS_TEXTURE_FILE_FORMAT_INVALID = 892
    QERR_VIS_TEXTURE_FILE_INVALID_COLOR_DEPTH = 893
    QERR_VIS_TEXTURE_FILE_INVALID_COMPRESSION = 894
    QERR_VIS_OPENGL_REQUIRED_EXTENSION_NOT_FOUND = 895
    QERR_VIS_MESH_INVALID_SHAPE_REFERENCE = 896
    QERR_VIS_MESH_INVALID_FACE_COUNT = 897
    QERR_VIS_MESH_INVALID_STRIDE = 898
    QERR_VIS_MESH_INVALID_NUMBER_OF_ATTRIBUTE_ELEMENTS = 899
    QERR_VIS_MESH_INVALID_NUMBER_OF_INDICES = 900
    QERR_VIS_MESH_INVALID_NUMBER_OF_NORMALS = 901
    QERR_VIS_MESH_INVALID_NUMBER_OF_TEXTURE_COORDINATES = 902
    QERR_VIS_MESH_INVALID_NUMBER_OF_VERTICES = 903
    QERR_VIS_MESH_INVALID_INDICES_INDEX = 904
    QERR_VIS_MESH_ATTRIBUTE_NOT_EQUAL_TO_OTHER_VERTICES = 905
    QERR_VIS_SHADER_VERTEX_NOT_SET = 906
    QERR_VIS_SHADER_FRAGMENT_NOT_SET = 907
    QERR_VIS_SHADER_VERTEX_COMPILE_FAILED = 908
    QERR_VIS_SHADER_FRAGMENT_COMPILE_FAILED = 909
    QERR_VIS_SHADER_NOT_COMPILED = 910
    QERR_VIS_OLD_SCENE_FILE_VERSION = 911
    QERR_VIS_UNRECOGNIZED_SCENE_FILE_VERSION = 912
    QERR_VIS_SCENE_BACKGROUND_COLOR_INVALID = 913
    QERR_VIS_SCENE_AMBIENT_COLOR_INVALID = 914
    QERR_VIS_SCENE_FRAMERATE_INVALID = 915
    QERR_VIS_SCENE_WIDTH_INVALID = 916
    QERR_VIS_SCENE_HEIGHT_INVALID = 917
    QERR_VIS_SCENE_MESH_ID_DUPLICATE = 918
    QERR_VIS_SCENE_MESH_ATTRIBUTE_ID_DUPLICATE = 919
    QERR_VIS_SCENE_TEXTURE_ID_DUPLICATE = 920
    QERR_VIS_SCENE_SHADER_ID_DUPLICATE = 921
    QERR_VIS_SCENE_SHADER_VARIABLE_ID_DUPLICATE = 922
    QERR_VIS_SCENE_SHADER_ATTRIBUTE_ID_DUPLICATE = 923
    QERR_VIS_SCENE_NEAR_CLIPPING_INVALID = 924
    QERR_VIS_SCENE_FAR_CLIPPING_INVALID = 925
    QERR_VIS_SCENE_NEAR_FAR_COMBINATION_INVALID = 926
    QERR_VIS_SCENE_VIEW_ANGLE_INVALID = 927
    QERR_VIS_SCENE_KEYBOARD_CAMERA_CONTROL_KEY_INVALID = 928
    QERR_VIS_SCENE_MOUSE_CAMERA_CONTROL_KEY_INVALID = 929
    QERR_VIS_SCENE_GESTURE_CAMERA_CONTROL_KEY_INVALID = 930
    QERR_VIS_SCENE_FOG_MODE_INVALID = 931
    QERR_VIS_SCENE_FOG_DENSITY_INVALID = 932
    QERR_VIS_SCENE_FOG_COLOR_INVALID = 933
    QERR_VIS_SCENE_FOG_START_INVALID = 934
    QERR_VIS_SCENE_FOG_END_INVALID = 935
    QERR_VIS_SCENE_FOG_START_OR_END_UNDEFINED = 936
    QERR_VIS_SCENE_OPENGL_VERSION_INVALID = 937
    QERR_VIS_SCENE_OBJECT_REFERENCES_UNKNOWN_MESH_ID = 938
    QERR_VIS_SCENE_OBJECT_REFERENCES_UNKNOWN_TEXTURE_ID = 939
    QERR_VIS_SCENE_OBJECT_ID_DUPLICATE = 940
    QERR_VIS_SCENE_ACTOR_REFERENCES_UNKNOWN_MESH_ID = 941
    QERR_VIS_SCENE_ACTOR_REFERENCES_UNKNOWN_TEXTURE_ID = 942
    QERR_VIS_SCENE_ACTOR_REFERENCES_UNKNOWN_OBJECT_ID = 943
    QERR_VIS_SCENE_ACTOR_REFERENCES_UNKNOWN_SHADER_ID = 944
    QERR_VIS_SCENE_ACTOR_REFERENCES_UNKNOWN_SHADER_VARIABLE_ID = 945
    QERR_VIS_SCENE_ACTOR_MISSING_SHADER_ID = 946
    QERR_VIS_SCENE_ACTOR_ID_DUPLICATE = 947
    QERR_VIS_SCENE_ACTOR_TYPE_UNKNOWN = 948
    QERR_VIS_SCENE_ACTOR_POSITION_INVALID = 949
    QERR_VIS_SCENE_ACTOR_SCALE_INVALID = 950
    QERR_VIS_SCENE_ACTOR_FOG_INVALID = 951
    QERR_VIS_SCENE_ACTOR_ORIENTATION_INVALID = 952
    QERR_VIS_SCENE_ACTOR_CHILD_AND_SIBLING_PROCESSING_ERROR = 953
    QERR_VIS_SCENE_ACTOR_INHERITANCE_INVALID = 954
    QERR_VIS_SCENE_ACTOR_COLOR_INVALID = 955
    QERR_VIS_SCENE_ACTOR_EMISSIVE_INVALID = 956
    QERR_VIS_SCENE_ACTOR_SPECULAR_INVALID = 957
    QERR_VIS_SCENE_ACTOR_SHININESS_INVALID = 958
    QERR_VIS_SCENE_ACTOR_ID_NOT_FOUND = 959
    QERR_VIS_ACTOR_MESH_POOL_NOT_INITIALIZED = 960
    QERR_VIS_ACTOR_TEXTURE_POOL_NOT_INITIALIZED = 961
    QERR_VIS_ACTOR_MESH_OUTSIDE_OF_MESH_POOL_BOUNDS = 962
    QERR_VIS_ACTOR_TEXTURE_OUTSIDE_OF_TEXTURE_POOL_BOUNDS = 963
    QERR_VIS_WRONG_MAGIC_NUMBER = 964
    QERR_VIS_COMM_ACTOR_REFERENCE_NOT_VALID = 965
    QERR_VIS_COMM_ACTOR_PARAMETER_NOT_VALID = 966
    QERR_VIS_COMM_DATA_STREAM_PAYLOAD_NOT_OF_EXPECTED_SIZE = 967
    QERR_VIS_COMM_RECEIVE_BUFFER_TOO_SMALL = 968
    QERR_VIS_COMM_RECEIVE_BUFFER_SIZE_INVALID = 969
    QERR_VIS_COMM_SEND_BUFFER_SIZE_INVALID = 970
    QERR_VIS_COMM_CONFIGURATION_PAYLOAD_NOT_OF_EXPECTED_SIZE = 971
    QERR_VIS_OLDER_QUARC_VERSION = 972
    QERR_MULTISAMPLE_OPERATION_NOT_SUPPORTED_DURING_DECIMATED_SAMPLING = 973
    QERR_HIL_SET_ANALOG_TERMINATION_STATE_NOT_SUPPORTED = 974
    QERR_HIL_SET_DIGITAL_TERMINATION_STATE_NOT_SUPPORTED = 975
    QERR_HIL_SET_PWM_TERMINATION_STATE_NOT_SUPPORTED = 976
    QERR_HIL_SET_OTHER_TERMINATION_STATE_NOT_SUPPORTED = 977
    QERR_HIL_AT_LEAST_ONE_CHANNEL_REQUIRED = 978
    QERR_HIL_INVALID_DIGITAL_DIRECTIONS = 979
    QERR_HIL_WRITE_TERMINATION_STATES_NOT_SUPPORTED = 980
    QERR_RCP_INVALID_SMOOTH_GEN_FREQUENCY = 981
    QERR_RCP_INVALID_SMOOTH_GEN_AMPLITUDE = 982
    QERR_RCP_INVALID_SMOOTH_GEN_INPUT_SIZE = 983
    QERR_RCP_INVALID_SIGMOID_SAMPLE_TIME = 984
    QERR_MISMATCHED_SCHEDULING_POLICY = 985
    QERR_HIL_DRIVER_NOT_FOUND = 986
    QERR_HIL_UNABLE_TO_READ_BITFILE = 987
    QERR_HIL_INVALID_FPGA_SIGNATURE = 988
    QERR_HIL_INVALID_RESOURCE_NAME = 989
    QERR_FORCE_DIMENSION_OPEN_FAILED = 990
    QERR_FORCE_DIMENSION_READ_FAILED = 991
    QERR_FORCE_DIMENSION_WRITE_FAILED = 992
    QERR_FORCE_DIMENSION_CLOSE_FAILED = 993
    QERR_FORCE_DIMENSION_NOT_CALIBRATED = 994
    QERR_NO_FORCE_DIMENSION_LICENSE = 995
    QERR_RCP_CHASSIS_NOT_FOUND = 996
    QERR_RCP_CHASSIS_NOT_SUPPORTED = 997
    QERR_RCP_INCORRECT_MODULE = 998
    QERR_RCP_MODULE_IO_ERROR = 999
    QERR_FPGA_ALREADY_RUNNING = 1000
    QERR_RESOURCE_NOT_INITIALIZED = 1001
    QERR_CORRUPT_FPGA_BITFILE = 1002
    QERR_FPGA_BUSY = 1003
    QERR_FPGA_BUSY_C_API = 1004
    QERR_FPGA_BUSY_SCAN_INTERFACE = 1005
    QERR_FPGA_BUSY_FPGA_INTERFACE = 1006
    QERR_FPGA_BUSY_INTERACTIVE = 1007
    QERR_FPGA_BUSY_EMULATION = 1008
    QERR_QBUS_NO_MODULES_FOUND = 1009
    QERR_QBUS_UNRECOGNIZED_MODULE = 1010
    QERR_SPI_INSUFFICIENT_BYTES_TO_SEND_AND_RECEIVE = 1011
    QERR_PARITY_NOT_SUPPORTED = 1012
    QERR_HARDWARE_FLOW_CONTROL_NOT_SUPPORTED = 1013
    QERR_SOFTWARE_FLOW_CONTROL_NOT_SUPPORTED = 1014
    QERR_DTR_DSR_NOT_SUPPORTED = 1015
    QERR_PARITY_VALUE_NOT_SUPPORTED = 1016
    QERR_STOP_BITS_VALUE_NOT_SUPPORTED = 1017
    QERR_WORD_LENGTH_VALUE_NOT_SUPPORTED = 1018
    QERR_NO_DATA_TO_SEND = 1019
    QERR_RCP_MISSING_MODULE = 1020
    QERR_RCP_MISSING_ANALOG_INPUT_MODULE = 1021
    QERR_RCP_MISSING_ANALOG_OUTPUT_MODULE = 1022
    QERR_RCP_MISSING_ENCODER_INPUT_MODULE = 1023
    QERR_RCP_MISSING_PWM_OUTPUT_MODULE = 1024
    QERR_RCP_MISSING_DIGITAL_INPUT_MODULE = 1025
    QERR_RCP_MISSING_DIGITAL_OUTPUT_MODULE = 1026
    QERR_RCP_MISSING_OTHER_INPUT_MODULE = 1027
    QERR_RCP_MISSING_OTHER_OUTPUT_MODULE = 1028
    QERR_DEVICE_NOT_CONNECTED = 1029
    QERR_TOO_MANY_PROPERTIES = 1030
    QERR_PSTREAM_NOT_VARIABLE_SIZE = 1031
    QERR_PSTREAM_VARIABLE_SIZE = 1032
    QERR_INVALID_DIMENSIONS = 1033
    QERR_LEAPMOTION_NOT_FOUND = 1034
    QERR_CANNOT_RESET_ENCODER_TO_NONZERO_VALUE = 1035
    QERR_ENCODER_INPUT_ERROR = 1036
    QERR_STALL_OCCURRED = 1037
    QERR_ONLY_I2C_MASTER_MODE_SUPPORTED = 1038
    QERR_NO_DEVICE_ADDRESS = 1039
    QERR_NO_ACKNOWLEDGEMENT = 1040
    QERR_NO_KINECT_SENSOR = 1041
    QERR_INVALID_KINECT = 1042
    QERR_KINECT_NOT_FOUND = 1043
    QERR_KINECT_FEATURE_NOT_ENABLED = 1044
    QERR_KINECT_NOT_INITIALIZED = 1045
    QERR_KINECT_ALREADY_INITIALIZED = 1046
    QERR_RESOLUTION_NOT_SUPPORTED = 1047
    QERR_NO_SPI_CHIP_SELECT = 1048
    QERR_MULTIPLE_SPI_CHIP_SELECTS = 1049
    QERR_UNSUPPORTED_I2C_OPERATION = 1050
    QERR_DIGITAL_EXPIRATION_STATE_NOT_ZERO = 1051
    QERR_PWM_EXPIRATION_STATE_NOT_ZERO = 1052
    QERR_OTHER_EXPIRATION_STATE_NOT_ZERO = 1053
    QERR_KINECT_NOT_SUPPORTED = 1054
    QERR_INVALID_MICO = 1055
    QERR_MICO_INVALID_JOINT = 1056
    QERR_MICO_MESSAGE_INVALID = 1057
    QERR_MICO_FIRMWARE_VERSION_NOT_READ = 1058
    QERR_MICO_FIRMWARE_VERSION_NOT_SUPPORTED = 1059
    QERR_MICO_ERROR = 1060
    QERR_MICO_ERROR_CANNOT_READ = 1061
    QERR_MICO_ERROR_CANNOT_WRITE = 1062
    QERR_INVALID_ROS = 1063
    QERR_ROS_ERROR = 1064
    QERR_INVALID_ROS_TOPIC = 1065
    QERR_ROS_SHUTDOWN = 1066
    QERR_ROS_INIT_ERROR = 1067
    QERR_INVALID_BOARD_VERSION = 1068
    QERR_UNABLE_TO_PROGRAM_FIRMWARE = 1069
    QERR_WRONG_NUMBER_OF_BYTES_RECEIVED = 1070
    QERR_INCOMPATIBLE_HARDWARE_VERSION = 1071
    QERR_INCOMPATIBLE_FIRMWARE_IMAGE = 1072
    QERR_BOARD_ALREADY_OPEN = 1073
    QERR_UNSUPPORTED_VIDEO_FORMAT = 1074
    QERR_INVALID_PACKET = 1075
    QERR_INVALID_CHECKSUM = 1076
    QERR_NEWER_VERSION_INSTALLED = 1077
    QERR_INVALID_ALIGNMENT_TYPE = 1078
    QERR_INVALID_ALLOCATION_CHUNK = 1079
    QERR_INVALID_BUFFER_MODE = 1080
    QERR_INVALID_COMPONENT_ID = 1081
    QERR_INVALID_CROP_REQUEST = 1082
    QERR_DCT_COEFFICIENT_OUT_OF_RANGE = 1083
    QERR_IDCT_SIZE_NOT_SUPPORTED = 1084
    QERR_MISMATCHED_SAMPLING_RATIO = 1085
    QERR_INVALID_HUFFMAN_TABLE = 1086
    QERR_INVALID_INPUT_COLORSPACE = 1087
    QERR_INVALID_JPEG_COLORSPACE = 1088
    QERR_INVALID_MARKER_LENGTH = 1089
    QERR_INVALID_MCU_SIZE = 1090
    QERR_INVALID_POOL_ID = 1091
    QERR_INVALID_PRECISION = 1092
    QERR_INVALID_PROGRESSION = 1093
    QERR_INVALID_PROGRESSIVE_SCRIPT = 1094
    QERR_INVALID_SAMPLING = 1095
    QERR_INVALID_SCAN_SCRIPT = 1096
    QERR_INVALID_LIBRARY_STATE = 1097
    QERR_INVALID_STRUCT_SIZE = 1098
    QERR_INVALID_VIRTUAL_ACCESS = 1099
    QERR_CANNOT_SUSPEND = 1100
    QERR_CCIR601_NOT_IMPLEMENTED = 1101
    QERR_COLOR_COMPONENT_COUNT = 1102
    QERR_COLOR_CONVERSION_NOT_IMPLEMENTED = 1103
    QERR_INVALID_DAC_INDEX = 1104
    QERR_INVALID_DAC_VALUE = 1105
    QERR_INVALID_DHT_INDEX = 1106
    QERR_INVALID_DQT_INDEX = 1107
    QERR_EMPTY_IMAGE = 1108
    QERR_EMS_READ_FAILED = 1109
    QERR_EMS_WRITE_FAILED = 1110
    QERR_END_OF_INPUT_EXPECTED = 1111
    QERR_FILE_READ_FAILED = 1112
    QERR_FILE_WRITE_FAILED = 1113
    QERR_FRACTIONAL_SAMPLING_NOT_IMPLEMENTED = 1114
    QERR_HUFFMAN_TABLE_OVERFLOW = 1115
    QERR_HUFFMAN_MISSING_CODE = 1116
    QERR_IMAGE_TOO_BIG = 1117
    QERR_MISMATCHED_QUANTIZATION_TABLE = 1118
    QERR_MISSING_SCAN_DATA = 1119
    QERR_COLOR_MODE_CHANGE_INVALID = 1120
    QERR_FEATURE_NOT_COMPILED = 1121
    QERR_NO_ARITHMETIC_TABLE = 1122
    QERR_BACKING_STORE_NOT_SUPPORTED = 1123
    QERR_NO_HUFFMAN_TABLE = 1124
    QERR_NO_QUANTIZATION_TABLE = 1125
    QERR_INVALID_FILE_TYPE = 1126
    QERR_TOO_MANY_QUANTIZATION_COMPONENTS = 1127
    QERR_CANNOT_QUANTIZE_TO_FEW_COLORS = 1128
    QERR_CANNOT_QUANTIZE_TO_MANY_COLORS = 1129
    QERR_SOF_DUPLICATE = 1130
    QERR_NO_SOS_MARKER = 1131
    QERR_SOF_NOT_SUPPORTED = 1132
    QERR_SOI_DUPLICATE = 1133
    QERR_SOS_BEFORE_SOF = 1134
    QERR_CANNOT_CREATE_TEMPORARY_FILE = 1135
    QERR_CANNOT_READ_TEMPORARY_FILE = 1136
    QERR_CANNOT_SEEK_TEMPORARY_FILE = 1137
    QERR_CANNOT_WRITE_TEMPORARY_FILE = 1138
    QERR_TOO_LITTLE_DATA = 1139
    QERR_MARKER_NOT_SUPPORTED = 1140
    QERR_VIRTUAL_ARRAY_BUG = 1141
    QERR_IMAGE_TOO_WIDE = 1142
    QERR_XMS_READ_FAILED = 1143
    QERR_XMS_WRITE_FAILED = 1144
    QERR_NO_DESTINATION_SET = 1145
    QERR_COMPRESSED_IMAGE_TOO_LARGE = 1146
    QERR_HIL_NAME_NOT_ASSIGNED = 1147
    QERR_HIL_SET_ANALOG_INPUT_CONFIGURATION_NOT_SUPPORTED = 1148
    QERR_MISSING_ANALOG_INPUT_CONFIGURATION = 1149
    QERR_INVALID_ANALOG_INPUT_CONFIGURATION = 1150
    QERR_INCOMPATIBLE_HARDWARE = 1151
    QERR_BAUD_RATE_EXCEEDS_MAXIMUM = 1152
    QERR_MISMATCHED_PWM_PERIOD_IN_BANK = 1153
    QERR_CALIBRATION_FAILED = 1154
    QERR_INVALID_I2C_STATE = 1155
    QERR_PARITY_ERROR = 1156
    QERR_FRAMING_ERROR = 1157
    QERR_FILE_TOO_LARGE = 1158
    QERR_INVALID_MEDIA_TYPE = 1159
    QERR_DEVICE_DISCONNECTED = 1160
    QERR_OS_ERROR = 1161
    QERR_WRONG_CALL_SEQUENCE = 1162
    QERR_DEVICE_RECOVERING = 1163
    QERR_DEVICE_IO_ERROR = 1164
    QERR_PROPERTY_IS_READ_ONLY = 1165
    QERR_IMAGE_STREAM_NOT_FOUND = 1166
    QERR_MISSING_REALSENSE = 1167
    QERR_EMITTER_CANNOT_BE_DISABLED = 1168
    QERR_INVALID_CAMERA_PROPERTY_VALUE = 1169
    QERR_INVALID_STRIDE = 1170
    QERR_INVALID_FILE_HANDLE = 1171
    QERR_BAROMETER_NOT_RESPONDING = 1172
    QERR_MAGNETOMETER_NOT_RESPONDING = 1173
    QERR_CONFLICTING_DIGITAL_MODES = 1174
    QERR_ELVISIII_TOP_BOARD_NO_POWER = 1175
    QERR_ELVISIII_EEPROM_ERROR = 1176
    QERR_ELVISIII_TOP_BOARD_INCOMPATIBLE = 1177
    QERR_NO_ELVISIII_LICENSE = 1178
    QERR_NO_RIO_GENERIC_LICENSE = 1179
    QERR_BORDER_TYPE_NOT_SUPPORTED = 1180
    QERR_FILTER_MASK_SIZE_NOT_SUPPORTED = 1181
    QERR_INVALID_ALGORITHM_HINT = 1182
    QERR_INVALID_ROUNDING_MODE = 1183
    QERR_INVALID_DATA_TYPE = 1184
    QERR_CANNOT_AUTODETECT_DSM_EXTERNAL = 1185
    QERR_PROPERTY_NOT_SUPPORTED = 1186
    QERR_CANNOT_INITIALIZE_OPENVR = 1187
    QERR_HIL_TASK_SET_BUFFER_OVERFLOW_MODE_NOT_SUPPORTED = 1188
    QERR_HIL_TASK_GET_BUFFER_OVERFLOWS_NOT_SUPPORTED = 1189
    QERR_MACRO_NOT_TERMINATED = 1190
    QERR_INVALID_MACRO_NAME = 1191
    QERR_UNSUPPORTED_IMAGE_CONVERSION = 1192
    QERR_CANNOT_CONVERT_CHARACTER = 1193
    QERR_NO_DEVICE = 1194
    QERR_PROTOCOL_BUFFER_TOO_SMALL = 1195
    QERR_INVALID_CALIBRATION = 1196
    QERR_RANGING_SENSOR_ERROR = 1197
    QERR_IO_ERROR = 1198
    QERR_DIVISION_BY_ZERO = 1199
    QERR_DEVICE_INITIALIZATION_FAILED = 1200
    QERR_DEVICE_DRIVER_INCOMPATIBLE = 1201
    QERR_HARDWARE_FAILURE = 1202
    QERR_SCALING_LOSES_ASPECT_RATIO = 1203
    QERR_SCALE_FACTOR_NOT_SUPPORTED = 1204
    QERR_BUFFER_TOO_SMALL = 1205
    QERR_INVALID_REALSENSE_VERSION = 1206
    QERR_INVALID_JSON = 1207
    QERR_NO_CODEC_FOUND = 1208
    QERR_CANNOT_START_XMLRPC_SERVER = 1209
    QERR_CANNOT_START_XMLRPC_CLIENT = 1210
    QERR_CANNOT_TALK_TO_ROS_MASTER = 1211
    QERR_INVALID_ROS_MASTER_RESPONSE = 1212
    QERR_ROS_MASTER_CALLER_ERROR = 1213
    QERR_ROS_MASTER_CALLER_FAILURE = 1214
    QERR_INVALID_ROS_SLAVE_REQUEST = 1215
    QERR_UNSUPPORTED_ROS_PROTOCOL = 1216
    QERR_ROS_NOT_ACTIVE = 1217
    QERR_CAMERA_IN_USE = 1218
    QERR_MUTEX_ALREADY_EXISTS = 1219
    QERR_MUTEX_ABANDONED = 1220
    QERR_MUTEX_NOT_FOUND = 1221
    QERR_IMAGE_DATA_TYPE_NOT_SUPPORTED = 1222
    QERR_PROTOCOL_DRIVER_NOT_FOUND = 1223
    QERR_CPU_GPIO_IN_USE = 1224
    QERR_OPTITRACK_LIBRARY_OPEN_FAILED = 1225
    QERR_OPTITRACK_UNSUPPORTED_API_FUNCTION = 1226
    QERR_OPTITRACK_UNSUPPORTED_RIGID_BODY_DEF_FILE = 1227
    QERR_OPTITRACK_INVALID_PROFILE_FILE = 1228
    QERR_UNABLE_TO_LOAD_CUDA_KERNEL = 1229
    QERR_UNABLE_TO_GET_CUDA_FUNCTION = 1230
    QERR_CAN_BUS_IDENTIFIER_TOO_LARGE = 1231
    QERR_INVALID_IPV6_ADDRESS = 1232
    QERR_CANNOT_GET_CAMERA_PROPERTIES = 1233
    QERR_INVALID_CUDA_CONTEXT = 1234
    QERR_MAP_FAILED = 1235
    QERR_RESOURCE_IN_USE = 1236
    QERR_CUDA_CONTEXT_IN_USE = 1237
    QERR_CUDA_COMPILATION_FAILED = 1238
    QERR_INVALID_GRAPHICS_CONTEXT = 1239
    QERR_UNRECOVERABLE_ERROR = 1240
    QERR_INCOMPATIBLE_TEXTURING_MODE = 1241
    QERR_INVALID_PEER_ACCESS = 1242
    QERR_OBJECT_ALREADY_INITIALIZED = 1243
    QERR_TOO_MANY_CUDA_BLOCKS = 1244
    QERR_MISSING_FGLOVE = 1245
    QERR_MISSING_FORCE_DIMENSION = 1246
    QERR_MISSING_FLY_CAPTURE = 1247
    QERR_MISSING_PCAN = 1248
    QERR_MISSING_VICON = 1249
    QERR_MISSING_LEAPMOTION = 1250
    QERR_MISSING_CANPCI = 1251
    QERR_MISSING_SCHUNK = 1252
    QERR_MISSING_FALCON = 1253
    QERR_NO_THREAD_CANCELLATION_STATE = 1254
    QERR_MISSING_OPENVR = 1255
    QERR_UNSUPPORTED_AUDIO_FORMAT = 1256
    QERR_CORRUPT_FILE = 1257
    QERR_WRONG_MODE_FOR_TRIGGERING = 1258
    QERR_CAMERA_NAME_NOT_ASSIGNED = 1259
    QERR_NOT_HIL_PROXY = 1260
    QERR_NOT_VIDEO3D_PROXY = 1261
    QERR_NO_DEVICE_SIMULATION_LICENSE = 1262
    QERR_NO_HIL_PROXY = 1263
    QERR_PRODUCT_NOT_IN_LICENSE_FILE = 1264
    QERR_BUFFER_MODE_NOT_SUPPORTED = 1265
    QERR_MISSING_FORMAT_SPECIFIER = 1266
    QERR_INVALID_FORMAT_RESTRICTION = 1267
    QERR_INVALID_TIMER_SEMAPHORE = 1268
    QERR_NOT_VIDEO_PROXY = 1269
    QERR_SAMPLES_LOST = 1270
    QERR_QARM_COMM_FAILURE_J0 = 1271
    QERR_QARM_COMM_FAILURE_J1_MASTER = 1272
    QERR_QARM_COMM_FAILURE_J1_SLAVE = 1273
    QERR_QARM_COMM_FAILURE_J2 = 1274
    QERR_QARM_COMM_FAILURE_J3 = 1275
    QERR_QARM_COMM_FAILURE_END_EFFECTOR = 1276
    QERR_QARM_COMM_FAILURE_GRIPPER = 1277
    QERR_QARM_OVERHEATING_J0 = 1278
    QERR_QARM_OVERHEATING_J1_MASTER = 1279
    QERR_QARM_OVERHEATING_J1_SLAVE = 1280
    QERR_QARM_OVERHEATING_J2 = 1281
    QERR_QARM_OVERHEATING_J3 = 1282
    QERR_QARM_OVERHEATING_GRIPPER = 1283
    QERR_QARM_OVERLOAD_J0 = 1284
    QERR_QARM_OVERLOAD_J1_MASTER = 1285
    QERR_QARM_OVERLOAD_J1_SLAVE = 1286
    QERR_QARM_OVERLOAD_J2 = 1287
    QERR_QARM_OVERLOAD_J3 = 1288
    QERR_QARM_OVERLOAD_GRIPPER = 1289
    QERR_QARM_MOTOR_ENCODER_J0 = 1290
    QERR_QARM_MOTOR_ENCODER_J1_MASTER = 1291
    QERR_QARM_MOTOR_ENCODER_J1_SLAVE = 1292
    QERR_QARM_MOTOR_ENCODER_J2 = 1293
    QERR_QARM_MOTOR_ENCODER_J3 = 1294
    QERR_QARM_MOTOR_ENCODER_GRIPPER = 1295
    QERR_QARM_ELECTRICAL_SHOCK_J0 = 1296
    QERR_QARM_ELECTRICAL_SHOCK_J1_MASTER = 1297
    QERR_QARM_ELECTRICAL_SHOCK_J1_SLAVE = 1298
    QERR_QARM_ELECTRICAL_SHOCK_J2 = 1299
    QERR_QARM_ELECTRICAL_SHOCK_J3 = 1300
    QERR_QARM_ELECTRICAL_SHOCK_GRIPPER = 1301
    QERR_QARM_INPUT_VOLTAGE_J0 = 1302
    QERR_QARM_INPUT_VOLTAGE_J1_MASTER = 1303
    QERR_QARM_INPUT_VOLTAGE_J1_SLAVE = 1304
    QERR_QARM_INPUT_VOLTAGE_J2 = 1305
    QERR_QARM_INPUT_VOLTAGE_J3 = 1306
    QERR_QARM_INPUT_VOLTAGE_GRIPPER = 1307
    QERR_PDU_SIZE_TOO_SMALL = 1308
    QERR_CANNOT_NEGOTIATE_PDU = 1309
    QERR_JOB_PENDING = 1310
    QERR_TOO_MANY_VARIABLES = 1311
    QERR_PDU_TOO_SMALL = 1312
    QERR_INVALID_PLC_ANSWER = 1313
    QERR_CANNOT_START_PLC = 1314
    QERR_PLC_ALREADY_RUN = 1315
    QERR_CANNOT_STOP_PLC = 1316
    QERR_CANNOT_COPY_RAM_TO_ROM = 1317
    QERR_CANNOT_COMPRESS = 1318
    QERR_PLC_ALREADY_STOPPED = 1319
    QERR_UPLOAD_FAILED = 1320
    QERR_INVALID_DATA_SIZE_RECEIVED = 1321
    QERR_INVALID_BLOCK_TYPE = 1322
    QERR_INVALID_BLOCK_NUMBER = 1323
    QERR_INVALID_BLOCK_SIZE = 1324
    QERR_DOWNLOAD_FAILED = 1325
    QERR_BLOCK_INSERT_REFUSED = 1326
    QERR_BLOCK_DELETE_REFUSED = 1327
    QERR_INVALID_PASSWORD = 1328
    QERR_NO_PASSWORD_TO_SET_OR_CLEAR = 1329
    QERR_FUNCTION_REFUSED = 1330
    QERR_DESTROYING_OBJECT = 1331
    QERR_CANNOT_CHANGE_PARAMETER = 1332
    QERR_ILLEGAL_MULTIBYTE_CHARACTER = 1333
    QERR_ILLEGAL_SURROGATE_CHARACTER = 1334
    QERR_ILLEGAL_CONTROL_CHARACTER = 1335
    QERR_ILLEGAL_NON_CHARACTER = 1336
    QERR_MISSING_END_TAG_NAME = 1337
    QERR_UNEXPECTED_NULL_CHARACTER = 1338
    QERR_UNEXPECTED_QUESTION_MARK = 1339
    QERR_END_BEFORE_TAG = 1340
    QERR_END_IN_TAG = 1341
    QERR_INVALID_FIRST_CHARACTER_OF_NAME = 1342
    QERR_INVALID_ARRAY_ELEMENT_SEPARATOR = 1343
    QERR_FAILED_TO_PARSE_INTEGER = 1344
    QERR_FAILED_TO_PARSE_REAL_NUMBER = 1345
    QERR_MATRIX_NOT_INVERTIBLE = 1346
    QERR_FORCE_TORQUE_SENSOR_DISCONNECTED = 1347
    QERR_END_QUOTE_EXPECTED = 1348
    QERR_TEXT_MATRICES_NOT_SUPPORTED = 1349
    QERR_SPARSE_MATRICES_NOT_SUPPORTED = 1350
    QERR_MATRIX_TYPE_NOT_RECOGNIZED = 1351
    QERR_VARIABLE_NOT_FOUND = 1352
    QERR_COMPLEX_NUMBERS_NOT_SUPPORTED = 1353
    QERR_BYTE_ORDER_NOT_SUPPORTED = 1354
    QERR_NUMBER_OF_ROWS_TOO_SMALL = 1355
    QERR_NUMBER_OF_COLUMNS_TOO_SMALL = 1356
    QERR_INVALID_I2C_TIMING_CONFIG = 1357
    QERR_IMU_HARDWARE_ERROR = 1358
    QERR_CONFLICTING_DIGITAL_FUNCTIONS = 1359
    QERR_ANALOG_OUTPUT_IS_NAN = 1360
    QERR_PWM_OUTPUT_IS_NAN = 1361
    QERR_OTHER_OUTPUT_IS_NAN = 1362
    QERR_DOUBLE_PROPERTY_IS_NAN = 1363
    QERR_PWM_FREQUENCY_IS_NAN = 1364
    QERR_PWM_DUTY_CYCLE_IS_NAN = 1365
    QERR_PWM_DEADBAND_IS_NAN = 1366
    QERR_CLOCK_FREQUENCY_IS_NAN = 1367
    QERR_ENCODER_FILTER_FREQUENCY_IS_NAN = 1368
    QERR_TOO_MANY_POINTS = 1369
    QERR_STACK_OVERFLOW = 1370
    QERR_STACK_UNDERFLOW = 1371
    QERR_NO_REFERENCE_SCAN = 1372
    QERR_INVALID_SCAN = 1373
    QERR_THEME_INCOMPATIBLE_WITH_DECOLORIZATION = 1374
    QERR_DYNAMIXEL_FAILED_TO_INITIALIZE = 1375
    QERR_DYNAMIXEL_COMMUNICATION_ERROR = 1376
    QERR_DYNAMIXEL_COULD_NOT_OPEN_DEVICE = 1377
    QERR_DYNAMIXEL_COULD_NOT_START_DEVICE = 1378
    QERR_DYNAMIXEL_COULD_NOT_CREATE_READ_GROUP = 1379
    QERR_DYNAMIXEL_COULD_NOT_CREATE_WRITE_GROUP = 1380
    QERR_DYNAMIXEL_COULD_NOT_CHANGE_WRITE_GROUP = 1381
    QERR_DYNAMIXEL_INVALID_GOAL_PWM = 1382
    QERR_DYNAMIXEL_INVALID_GOAL_POSITION = 1383
    QERR_DYNAMIXEL_INVALID_GAINS = 1384
    QERR_DYNAMIXEL_INVALID_PROFILE_PARAMS = 1385
    QERR_CANNOT_CONNECT_TO_PLC = 1386
    QERR_FILE_ALREADY_EXISTS = 1387
    QERR_TOO_MANY_TELEMETRY = 1388
    QERR_DIGITAL_EXPIRATION_STATE_NOT_UNCHANGED = 1389
    QERR_VARIABLE_STEP_SOLVERS_NOT_SUPPORTED = 1390
    QERR_INVALID_NEWLINE_SEPARATED_STRING = 1391
    QERR_SENSOR_TYPE_NOT_SUPPORTED = 1392
    QERR_MISSING_SPINNAKER = 1393
    QERR_GENICAM_CANNOT_INITIALIZE_CAMERA = 1394
    QERR_GENICAM_CANNOT_GET_FEATURES = 1395
    QERR_GENICAM_CANNOT_READ_NODE = 1396
    QERR_GENICAM_CANNOT_WRITE_NODE = 1397
    QERR_GENICAM_CANNOT_FIND_CAMERA = 1398
    QERR_GENICAM_CANNOT_GET_CAMERA_NODE = 1399
    QERR_GENICAM_CANNOT_SET_IMAGE_SIZE = 1400
    QERR_GENICAM_CANNOT_SET_PIXEL_FORMAT = 1401
    QERR_GENICAM_CANNOT_SET_ANALOG_CONTROL = 1402
    QERR_GENICAM_CANNOT_SET_ACQUISITION_NODE = 1403
    QERR_GENICAM_CANNOT_GRAB_IMAGE = 1404
    QERR_GENICAM_GRAB_IMAGE_TIMEOUT = 1405
    QERR_GENICAM_CANNOT_CLOSE = 1406
    QERR_CONTINUOUS_THRESHOLD_EXCEEDS_PEEK = 1407
    QERR_PEER_CLOSED_CONNECTION = 1408
    QERR_USB_CANNOT_GET_CONFIG_DESCRIPTOR = 1409
    QERR_USB_CANNOT_CLAIM_INTERFACE = 1410
    QERR_NUMBER_OF_ERRORS = 1411
end

function create_directory(folder_path, permissions)
    ccall((:create_directory, hil_sdk), t_error, (Ptr{Cchar}, t_uint), folder_path, permissions)
end

function std_wopen(filename, open_flags, share_flags, mode, file_handle)
    ccall((:std_wopen, hil_sdk), t_error, (Ptr{Cwchar_t}, Cint, Cint, Cint, Ptr{Cint}), filename, open_flags, share_flags, mode, file_handle)
end

function std_stat(filename, buffer)
    ccall((:std_stat, hil_sdk), t_error, (Ptr{Cchar}, Ptr{Cvoid}), filename, buffer)
end

const t_stdfile = Ptr{Libc.FILE}

function stdfile_open(filename, mode, file_handle)
    ccall((:stdfile_open, hil_sdk), t_error, (Ptr{Cchar}, Ptr{Cchar}, Ptr{t_stdfile}), filename, mode, file_handle)
end

function stdfile_gets(file_handle, buffer, buffer_size)
    ccall((:stdfile_gets, hil_sdk), t_int, (t_stdfile, Ptr{Cchar}, Csize_t), file_handle, buffer, buffer_size)
end

function stdfile_putwc(file_handle, c)
    ccall((:stdfile_putwc, hil_sdk), t_int, (t_stdfile, Cwchar_t), file_handle, c)
end

const errno_t = Cint

const _off_t = off_t

@cenum tag_file_origin::UInt32 begin
    FILE_ORIGIN_BEGINNING = 0
    FILE_ORIGIN_CURRENT = 1
    FILE_ORIGIN_END = 2
end

const t_file_origin = tag_file_origin

function create_wdirectory(folder_path, permissions)
    ccall((:create_wdirectory, hil_sdk), t_error, (Ptr{Cwchar_t}, t_uint), folder_path, permissions)
end

function std_open(filename, open_flags, share_flags, mode, file_handle)
    ccall((:std_open, hil_sdk), t_error, (Ptr{Cchar}, Cint, Cint, Cint, Ptr{Cint}), filename, open_flags, share_flags, mode, file_handle)
end

function std_read(file_handle, buffer, count)
    ccall((:std_read, hil_sdk), Cssize_t, (Cint, Ptr{Cvoid}, Csize_t), file_handle, buffer, count)
end

function std_write(file_handle, buffer, count)
    ccall((:std_write, hil_sdk), Cssize_t, (Cint, Ptr{Cvoid}, Csize_t), file_handle, buffer, count)
end

const t_long = Int64

const t_int64 = t_long

function std_seek(file_handle, offset, origin)
    ccall((:std_seek, hil_sdk), t_error, (Cint, t_int64, t_file_origin), file_handle, offset, origin)
end

function std_close(file_handle)
    ccall((:std_close, hil_sdk), t_error, (Cint,), file_handle)
end

function std_fstat(file_handle, buffer)
    ccall((:std_fstat, hil_sdk), t_error, (Cint, Ptr{Cvoid}), file_handle, buffer)
end

function std_wstat(filename, buffer)
    ccall((:std_wstat, hil_sdk), t_error, (Ptr{Cwchar_t}, Ptr{Cvoid}), filename, buffer)
end

function std_futime(file_handle, times)
    ccall((:std_futime, hil_sdk), t_error, (Cint, Ptr{Cvoid}), file_handle, times)
end

function stdfile_wopen(filename, mode, file_handle)
    ccall((:stdfile_wopen, hil_sdk), t_error, (Ptr{Cwchar_t}, Ptr{Cwchar_t}, Ptr{t_stdfile}), filename, mode, file_handle)
end

function stdfile_read(file_handle, buffer, buffer_size, bytes_read)
    ccall((:stdfile_read, hil_sdk), t_error, (t_stdfile, Ptr{Cvoid}, Csize_t, Ptr{Csize_t}), file_handle, buffer, buffer_size, bytes_read)
end

function stdfile_write(file_handle, buffer, buffer_size, bytes_written)
    ccall((:stdfile_write, hil_sdk), t_error, (t_stdfile, Ptr{Cvoid}, Csize_t, Ptr{Csize_t}), file_handle, buffer, buffer_size, bytes_written)
end

function stdfile_wgets(file_handle, buffer, buffer_size)
    ccall((:stdfile_wgets, hil_sdk), t_int, (t_stdfile, Ptr{Cwchar_t}, Csize_t), file_handle, buffer, buffer_size)
end

function stdfile_putc(file_handle, c)
    ccall((:stdfile_putc, hil_sdk), t_int, (t_stdfile, Cchar), file_handle, c)
end

function stdfile_seek(file_handle, offset, origin)
    ccall((:stdfile_seek, hil_sdk), t_error, (t_stdfile, t_int64, t_file_origin), file_handle, offset, origin)
end

function stdfile_close(file_handle)
    ccall((:stdfile_close, hil_sdk), t_error, (t_stdfile,), file_handle)
end

function stdfile_temp(file_handle)
    ccall((:stdfile_temp, hil_sdk), t_error, (Ptr{t_stdfile},), file_handle)
end

function std_symlink(filename, target)
    ccall((:std_symlink, hil_sdk), t_error, (Ptr{Cchar}, Ptr{Cchar}), filename, target)
end

function msg_get_current_localeA(buffer, buffer_size)
    ccall((:msg_get_current_localeA, hil_sdk), t_error, (Ptr{Cchar}, Csize_t), buffer, buffer_size)
end

function msg_set_current_localeA(category, locale)
    ccall((:msg_set_current_localeA, hil_sdk), t_error, (Cint, Ptr{Cchar}), category, locale)
end

function msg_read_error_messageA(error_folder, error_filename, locale, error_code, buffer, length)
    ccall((:msg_read_error_messageA, hil_sdk), t_error, (Ptr{Cchar}, Ptr{Cchar}, Ptr{Cchar}, t_error, Ptr{Cchar}, Csize_t), error_folder, error_filename, locale, error_code, buffer, length)
end

function msg_get_error_messageA(locale, error_code, buffer, length)
    ccall((:msg_get_error_messageA, hil_sdk), t_error, (Ptr{Cchar}, t_error, Ptr{Cchar}, Csize_t), locale, error_code, buffer, length)
end

function msg_get_current_localeW(buffer, buffer_size)
    ccall((:msg_get_current_localeW, hil_sdk), t_error, (Ptr{Cwchar_t}, Csize_t), buffer, buffer_size)
end

function msg_set_current_localeW(category, locale)
    ccall((:msg_set_current_localeW, hil_sdk), t_error, (Cint, Ptr{Cwchar_t}), category, locale)
end

function msg_read_error_messageW(error_folder, error_filename, locale, error_code, buffer, length)
    ccall((:msg_read_error_messageW, hil_sdk), t_error, (Ptr{Cwchar_t}, Ptr{Cwchar_t}, Ptr{Cwchar_t}, t_error, Ptr{Cwchar_t}, Csize_t), error_folder, error_filename, locale, error_code, buffer, length)
end

function msg_get_error_messageW(locale, error_code, buffer, length)
    ccall((:msg_get_error_messageW, hil_sdk), t_error, (Ptr{Cwchar_t}, t_error, Ptr{Cwchar_t}, Csize_t), locale, error_code, buffer, length)
end

function msg_get_default_error_messageW(locale, error_code, buffer, length)
    ccall((:msg_get_default_error_messageW, hil_sdk), t_error, (Ptr{Cwchar_t}, t_error, Ptr{Cwchar_t}, Csize_t), locale, error_code, buffer, length)
end

function msg_get_default_error_messageA(locale, error_code, buffer, length)
    ccall((:msg_get_default_error_messageA, hil_sdk), t_error, (Ptr{Cchar}, t_error, Ptr{Cchar}, Csize_t), locale, error_code, buffer, length)
end

const qthread_mutexattr_t = pthread_mutexattr_t

const qthread_mutex_t = pthread_mutex_t

function qthread_mutex_init(mutex, attributes)
    ccall((:qthread_mutex_init, hil_sdk), t_error, (Ptr{qthread_mutex_t}, Ptr{qthread_mutexattr_t}), mutex, attributes)
end

function qthread_mutex_create(mutex, name, exclusive)
    ccall((:qthread_mutex_create, hil_sdk), t_error, (Ptr{Ptr{qthread_mutex_t}}, Ptr{Cchar}, t_boolean), mutex, name, exclusive)
end

function qthread_mutex_lock(mutex)
    ccall((:qthread_mutex_lock, hil_sdk), t_error, (Ptr{qthread_mutex_t},), mutex)
end

function qthread_mutex_unlock(mutex)
    ccall((:qthread_mutex_unlock, hil_sdk), t_error, (Ptr{qthread_mutex_t},), mutex)
end

function qthread_mutex_destroy(mutex)
    ccall((:qthread_mutex_destroy, hil_sdk), t_error, (Ptr{qthread_mutex_t},), mutex)
end

function qthread_mutex_close(mutex)
    ccall((:qthread_mutex_close, hil_sdk), t_error, (Ptr{qthread_mutex_t},), mutex)
end

const qpid_t = pid_t

function getqpid()
    ccall((:getqpid, hil_sdk), qpid_t, ())
end

struct tag_timeout
    seconds::t_long
    nanoseconds::t_int
    is_absolute::t_boolean
end

const t_timeout = tag_timeout

struct tag_resource_usage
    user_time::t_timeout
    system_time::t_timeout
end

const t_resource_usage = tag_resource_usage

function process_getrusage(usage)
    ccall((:process_getrusage, hil_sdk), t_error, (Ptr{t_resource_usage},), usage)
end

struct qsched_param
    sched_priority::Cint
end

const qsched_param_t = qsched_param

struct tag_qcpu_set
    mask::t_uint
end

const qcpu_set_t = tag_qcpu_set

function QCPU_ZERO(set)
    ccall((:QCPU_ZERO, hil_sdk), Cvoid, (Ptr{qcpu_set_t},), set)
end

function QCPU_SET(cpu, set)
    ccall((:QCPU_SET, hil_sdk), Cvoid, (Cint, Ptr{qcpu_set_t}), cpu, set)
end

function QCPU_CLR(cpu, set)
    ccall((:QCPU_CLR, hil_sdk), Cvoid, (Cint, Ptr{qcpu_set_t}), cpu, set)
end

function QCPU_ISSET(cpu, set)
    ccall((:QCPU_ISSET, hil_sdk), Cint, (Cint, Ptr{qcpu_set_t}), cpu, set)
end

@cenum tag_sleep_mode::UInt32 begin
    SLEEP_MODE_ENABLED = 0
    SLEEP_MODE_DISABLED = 1
end

const t_sleep_mode = tag_sleep_mode

function qsched_setaffinity(pid, cpu_set_size, set)
    ccall((:qsched_setaffinity, hil_sdk), Cint, (qpid_t, Csize_t, Ptr{qcpu_set_t}), pid, cpu_set_size, set)
end

function qsched_getaffinity(pid, cpu_set_size, set)
    ccall((:qsched_getaffinity, hil_sdk), Cint, (qpid_t, Csize_t, Ptr{qcpu_set_t}), pid, cpu_set_size, set)
end

function qsched_get_priority_min(policy)
    ccall((:qsched_get_priority_min, hil_sdk), Cint, (Cint,), policy)
end

function qsched_get_priority_max(policy)
    ccall((:qsched_get_priority_max, hil_sdk), Cint, (Cint,), policy)
end

function qsched_get_num_cpus()
    ccall((:qsched_get_num_cpus, hil_sdk), Cint, ())
end

function qsched_set_sleep_mode(mode)
    ccall((:qsched_set_sleep_mode, hil_sdk), t_error, (t_sleep_mode,), mode)
end

function qsched_yield()
    ccall((:qsched_yield, hil_sdk), t_error, ())
end

const qsigset_t = sigset_t

const qsigaction_t = sigaction

function qsigemptyset(set)
    ccall((:qsigemptyset, hil_sdk), t_error, (Ptr{qsigset_t},), set)
end

function qsigfillset(set)
    ccall((:qsigfillset, hil_sdk), t_error, (Ptr{qsigset_t},), set)
end

function qsigismember(set, signal_number)
    ccall((:qsigismember, hil_sdk), t_int, (Ptr{qsigset_t}, Cint), set, signal_number)
end

function qsigaddset(set, signal_number)
    ccall((:qsigaddset, hil_sdk), t_error, (Ptr{qsigset_t}, Cint), set, signal_number)
end

function qsigdelset(set, signal_number)
    ccall((:qsigdelset, hil_sdk), t_error, (Ptr{qsigset_t}, Cint), set, signal_number)
end

function qsigaction(signum, action, old_action)
    ccall((:qsigaction, hil_sdk), t_error, (Cint, Ptr{qsigaction_t}, Ptr{qsigaction_t}), signum, action, old_action)
end

function qthread_sigmask(how, set, old_set)
    ccall((:qthread_sigmask, hil_sdk), t_error, (Cint, Ptr{qsigset_t}, Ptr{qsigset_t}), how, set, old_set)
end

function qraise(signal_number)
    ccall((:qraise, hil_sdk), t_error, (Cint,), signal_number)
end

function string_length(source, source_size)
    ccall((:string_length, hil_sdk), Csize_t, (Ptr{Cchar}, Csize_t), source, source_size)
end

function string_copy(destination, destination_size, source)
    ccall((:string_copy, hil_sdk), t_error, (Ptr{Cchar}, Csize_t, Ptr{Cchar}), destination, destination_size, source)
end

function string_concatenate(destination, destination_size, source)
    ccall((:string_concatenate, hil_sdk), t_error, (Ptr{Cchar}, Csize_t, Ptr{Cchar}), destination, destination_size, source)
end

function string_find(source, source_size, c, new_size)
    ccall((:string_find, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Cchar, Ptr{Csize_t}), source, source_size, c, new_size)
end

function string_find_in_set(source, source_size, set, set_size, new_size, set_index)
    ccall((:string_find_in_set, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Ptr{Cchar}, Csize_t, Ptr{Csize_t}, Ptr{Csize_t}), source, source_size, set, set_size, new_size, set_index)
end

function string_find_substring(source, source_size, substring, new_size)
    ccall((:string_find_substring, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Ptr{Cchar}, Ptr{Csize_t}), source, source_size, substring, new_size)
end

function string_equal(source, maximum_size, comperand)
    ccall((:string_equal, hil_sdk), t_boolean, (Ptr{Cchar}, Csize_t, Ptr{Cchar}), source, maximum_size, comperand)
end

function string_equal_ignoring_case(source, maximum_size, comperand)
    ccall((:string_equal_ignoring_case, hil_sdk), t_boolean, (Ptr{Cchar}, Csize_t, Ptr{Cchar}), source, maximum_size, comperand)
end

function string_to_integer(source, source_size, value, new_size)
    ccall((:string_to_integer, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Ptr{t_int}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function string_to_double(source, source_size, value, new_size)
    ccall((:string_to_double, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Ptr{t_double}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function integer_to_string(destination, destination_size, base, value)
    ccall((:integer_to_string, hil_sdk), t_error, (Ptr{Cchar}, Csize_t, t_uint, t_int), destination, destination_size, base, value)
end

function unsigned_to_string(destination, destination_size, base, value)
    ccall((:unsigned_to_string, hil_sdk), t_error, (Ptr{Cchar}, Csize_t, t_uint, t_uint), destination, destination_size, base, value)
end

function string_replace(source, source_size, old_char, new_char)
    ccall((:string_replace, hil_sdk), t_error, (Ptr{Cchar}, Csize_t, Cchar, Cchar), source, source_size, old_char, new_char)
end

const t_ulong = UInt64

function string_to_unsigned_long(source, source_size, value, new_size)
    ccall((:string_to_unsigned_long, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Ptr{t_ulong}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function string_move(destination, destination_size, source)
    ccall((:string_move, hil_sdk), t_error, (Ptr{Cchar}, Csize_t, Ptr{Cchar}), destination, destination_size, source)
end

function string_to_long(source, source_size, value, new_size)
    ccall((:string_to_long, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Ptr{t_long}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function string_to_timeout(source, source_size, value, new_size)
    ccall((:string_to_timeout, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Ptr{t_timeout}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function string_to_boolean(source, source_size, value, new_size)
    ccall((:string_to_boolean, hil_sdk), Ptr{Cchar}, (Ptr{Cchar}, Csize_t, Ptr{t_boolean}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function string_to_wstring(source, source_size, destination, destination_size)
    ccall((:string_to_wstring, hil_sdk), t_error, (Ptr{Cchar}, Csize_t, Ptr{Cwchar_t}, Csize_t), source, source_size, destination, destination_size)
end

function wstring_length(source, source_size)
    ccall((:wstring_length, hil_sdk), Csize_t, (Ptr{Cwchar_t}, Csize_t), source, source_size)
end

function wstring_copy(destination, destination_size, source)
    ccall((:wstring_copy, hil_sdk), t_error, (Ptr{Cwchar_t}, Csize_t, Ptr{Cwchar_t}), destination, destination_size, source)
end

function wstring_move(destination, destination_size, source)
    ccall((:wstring_move, hil_sdk), t_error, (Ptr{Cwchar_t}, Csize_t, Ptr{Cwchar_t}), destination, destination_size, source)
end

function wstring_concatenate(destination, destination_size, source)
    ccall((:wstring_concatenate, hil_sdk), t_error, (Ptr{Cwchar_t}, Csize_t, Ptr{Cwchar_t}), destination, destination_size, source)
end

function wstring_find(source, source_size, c, new_size)
    ccall((:wstring_find, hil_sdk), Ptr{Cwchar_t}, (Ptr{Cwchar_t}, Csize_t, Cwchar_t, Ptr{Csize_t}), source, source_size, c, new_size)
end

function wstring_find_in_set(source, source_size, set, set_size, new_size, set_index)
    ccall((:wstring_find_in_set, hil_sdk), Ptr{Cwchar_t}, (Ptr{Cwchar_t}, Csize_t, Ptr{Cwchar_t}, Csize_t, Ptr{Csize_t}, Ptr{Csize_t}), source, source_size, set, set_size, new_size, set_index)
end

function wstring_find_substring(source, source_size, substring, new_size)
    ccall((:wstring_find_substring, hil_sdk), Ptr{Cwchar_t}, (Ptr{Cwchar_t}, Csize_t, Ptr{Cwchar_t}, Ptr{Csize_t}), source, source_size, substring, new_size)
end

function wstring_equal(source, maximum_size, comperand)
    ccall((:wstring_equal, hil_sdk), t_boolean, (Ptr{Cwchar_t}, Csize_t, Ptr{Cwchar_t}), source, maximum_size, comperand)
end

function wstring_equal_ignoring_case(source, maximum_size, comperand)
    ccall((:wstring_equal_ignoring_case, hil_sdk), t_boolean, (Ptr{Cwchar_t}, Csize_t, Ptr{Cwchar_t}), source, maximum_size, comperand)
end

function wstring_to_integer(source, source_size, value, new_size)
    ccall((:wstring_to_integer, hil_sdk), Ptr{Cwchar_t}, (Ptr{Cwchar_t}, Csize_t, Ptr{t_int}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function wstring_to_double(source, source_size, value, new_size)
    ccall((:wstring_to_double, hil_sdk), Ptr{Cwchar_t}, (Ptr{Cwchar_t}, Csize_t, Ptr{t_double}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function integer_to_wstring(destination, destination_size, base, value)
    ccall((:integer_to_wstring, hil_sdk), t_error, (Ptr{Cwchar_t}, Csize_t, t_uint, t_int), destination, destination_size, base, value)
end

function unsigned_to_wstring(destination, destination_size, base, value)
    ccall((:unsigned_to_wstring, hil_sdk), t_error, (Ptr{Cwchar_t}, Csize_t, t_uint, t_uint), destination, destination_size, base, value)
end

function wstring_to_unsigned_long(source, source_size, value, new_size)
    ccall((:wstring_to_unsigned_long, hil_sdk), Ptr{Cwchar_t}, (Ptr{Cwchar_t}, Csize_t, Ptr{t_ulong}, Ptr{Csize_t}), source, source_size, value, new_size)
end

function wstring_replace(source, source_size, old_char, new_char)
    ccall((:wstring_replace, hil_sdk), t_error, (Ptr{Cwchar_t}, Csize_t, Cwchar_t, Cwchar_t), source, source_size, old_char, new_char)
end

const qthread_t = pthread_t

const qthread_id_t = pthread_t

# typedef void ( * qthread_once_callback_t ) ( void )
const qthread_once_callback_t = Ptr{Cvoid}

const qthread_return_t = Ptr{Cvoid}

const qthread_once_t = pthread_once_t

const qthread_key_t = pthread_key_t

const qthread_attr_t = pthread_attr_t

# typedef void ( QTHREAD_DECL * qthread_key_destructor_t ) ( void * )
const qthread_key_destructor_t = Ptr{Cvoid}

function qthread_attr_init(attributes)
    ccall((:qthread_attr_init, hil_sdk), t_error, (Ptr{qthread_attr_t},), attributes)
end

function qthread_attr_getstacksize(attributes, stack_size)
    ccall((:qthread_attr_getstacksize, hil_sdk), t_error, (Ptr{qthread_attr_t}, Ptr{Csize_t}), attributes, stack_size)
end

function qthread_attr_setstacksize(attributes, stack_size)
    ccall((:qthread_attr_setstacksize, hil_sdk), t_error, (Ptr{qthread_attr_t}, Csize_t), attributes, stack_size)
end

function qthread_attr_getinheritsched(attributes, inherit_sched)
    ccall((:qthread_attr_getinheritsched, hil_sdk), t_error, (Ptr{qthread_attr_t}, Ptr{Cint}), attributes, inherit_sched)
end

function qthread_attr_setinheritsched(attributes, inherit_sched)
    ccall((:qthread_attr_setinheritsched, hil_sdk), t_error, (Ptr{qthread_attr_t}, Cint), attributes, inherit_sched)
end

function qthread_attr_getschedpolicy(attributes, policy)
    ccall((:qthread_attr_getschedpolicy, hil_sdk), t_error, (Ptr{qthread_attr_t}, Ptr{Cint}), attributes, policy)
end

function qthread_attr_setschedpolicy(attributes, policy)
    ccall((:qthread_attr_setschedpolicy, hil_sdk), t_error, (Ptr{qthread_attr_t}, Cint), attributes, policy)
end

function qthread_attr_getschedparam(attributes, param)
    ccall((:qthread_attr_getschedparam, hil_sdk), t_error, (Ptr{qthread_attr_t}, Ptr{qsched_param_t}), attributes, param)
end

function qthread_attr_setschedparam(attributes, param)
    ccall((:qthread_attr_setschedparam, hil_sdk), t_error, (Ptr{qthread_attr_t}, Ptr{qsched_param_t}), attributes, param)
end

function qthread_attr_getaffinity(attributes, cpu_set_size, set)
    ccall((:qthread_attr_getaffinity, hil_sdk), Cint, (Ptr{qthread_attr_t}, Csize_t, Ptr{qcpu_set_t}), attributes, cpu_set_size, set)
end

function qthread_attr_setaffinity(attributes, cpu_set_size, set)
    ccall((:qthread_attr_setaffinity, hil_sdk), Cint, (Ptr{qthread_attr_t}, Csize_t, Ptr{qcpu_set_t}), attributes, cpu_set_size, set)
end

function qthread_attr_destroy(attributes)
    ccall((:qthread_attr_destroy, hil_sdk), t_error, (Ptr{qthread_attr_t},), attributes)
end

function qthread_create(thread, thread_id, attributes, start_routine, argument)
    ccall((:qthread_create, hil_sdk), t_error, (Ptr{qthread_t}, Ptr{qthread_id_t}, Ptr{qthread_attr_t}, Ptr{Cvoid}, Ptr{Cvoid}), thread, thread_id, attributes, start_routine, argument)
end

function qthread_self()
    ccall((:qthread_self, hil_sdk), qthread_t, ())
end

function qthread_id()
    ccall((:qthread_id, hil_sdk), qthread_id_t, ())
end

function qthread_equal(t1, t2)
    ccall((:qthread_equal, hil_sdk), t_boolean, (qthread_id_t, qthread_id_t), t1, t2)
end

function qthread_cancel(thread, thread_id)
    ccall((:qthread_cancel, hil_sdk), t_error, (qthread_t, qthread_id_t), thread, thread_id)
end

function qthread_testcancel()
    ccall((:qthread_testcancel, hil_sdk), Cvoid, ())
end

function qthread_join(thread, exit_code)
    ccall((:qthread_join, hil_sdk), t_error, (qthread_t, Ptr{qthread_return_t}), thread, exit_code)
end

function qthread_timedjoin(thread, exit_code, timeout)
    ccall((:qthread_timedjoin, hil_sdk), t_error, (qthread_t, Ptr{qthread_return_t}, Ptr{t_timeout}), thread, exit_code, timeout)
end

function qthread_exit(exit_code)
    ccall((:qthread_exit, hil_sdk), Cvoid, (qthread_return_t,), exit_code)
end

function qthread_getschedparam(thread, policy, param)
    ccall((:qthread_getschedparam, hil_sdk), t_error, (qthread_t, Ptr{Cint}, Ptr{qsched_param_t}), thread, policy, param)
end

function qthread_setschedparam(thread, policy, param)
    ccall((:qthread_setschedparam, hil_sdk), t_error, (qthread_t, Cint, Ptr{qsched_param_t}), thread, policy, param)
end

function qthread_setaffinity(thread, cpu_set_size, set)
    ccall((:qthread_setaffinity, hil_sdk), Cint, (qthread_t, Csize_t, Ptr{qcpu_set_t}), thread, cpu_set_size, set)
end

function qthread_getrusage(usage)
    ccall((:qthread_getrusage, hil_sdk), t_error, (Ptr{t_resource_usage},), usage)
end

function qthread_once(once_control, init_routine)
    ccall((:qthread_once, hil_sdk), t_error, (Ptr{qthread_once_t}, qthread_once_callback_t), once_control, init_routine)
end

function qthread_key_create(key, destructor)
    ccall((:qthread_key_create, hil_sdk), t_error, (Ptr{qthread_key_t}, qthread_key_destructor_t), key, destructor)
end

function qthread_key_delete(key)
    ccall((:qthread_key_delete, hil_sdk), t_error, (qthread_key_t,), key)
end

function qthread_setspecific(key, value)
    ccall((:qthread_setspecific, hil_sdk), t_error, (qthread_key_t, Ptr{Cvoid}), key, value)
end

function qthread_getspecific(key)
    ccall((:qthread_getspecific, hil_sdk), Ptr{Cvoid}, (qthread_key_t,), key)
end

function qthread_setcancelstate(state, old_state)
    ccall((:qthread_setcancelstate, hil_sdk), t_error, (Cint, Ptr{Cint}), state, old_state)
end

function timeout_get_timeout(timeout, time_value)
    ccall((:timeout_get_timeout, hil_sdk), Ptr{t_timeout}, (Ptr{t_timeout}, Cdouble), timeout, time_value)
end

function timeout_is_negative(timeout)
    ccall((:timeout_is_negative, hil_sdk), t_boolean, (Ptr{t_timeout},), timeout)
end

function timeout_is_zero(timeout)
    ccall((:timeout_is_zero, hil_sdk), t_boolean, (Ptr{t_timeout},), timeout)
end

function timeout_is_expired(timeout)
    ccall((:timeout_is_expired, hil_sdk), t_boolean, (Ptr{t_timeout},), timeout)
end

function timeout_get_absolute(result, timeout)
    ccall((:timeout_get_absolute, hil_sdk), t_error, (Ptr{t_timeout}, Ptr{t_timeout}), result, timeout)
end

function timeout_get_relative(result, timeout)
    ccall((:timeout_get_relative, hil_sdk), t_error, (Ptr{t_timeout}, Ptr{t_timeout}), result, timeout)
end

function timeout_compare(left, right)
    ccall((:timeout_compare, hil_sdk), t_int, (Ptr{t_timeout}, Ptr{t_timeout}), left, right)
end

function timeout_subtract(result, left, right)
    ccall((:timeout_subtract, hil_sdk), t_error, (Ptr{t_timeout}, Ptr{t_timeout}, Ptr{t_timeout}), result, left, right)
end

function timeout_add(result, left, right)
    ccall((:timeout_add, hil_sdk), t_error, (Ptr{t_timeout}, Ptr{t_timeout}, Ptr{t_timeout}), result, left, right)
end

function timeout_get_current_time(time)
    ccall((:timeout_get_current_time, hil_sdk), t_error, (Ptr{t_timeout},), time)
end

function timeout_get_high_resolution_time(time)
    ccall((:timeout_get_high_resolution_time, hil_sdk), t_error, (Ptr{t_timeout},), time)
end

function timeout_get_thread_cpu_time(time)
    ccall((:timeout_get_thread_cpu_time, hil_sdk), t_error, (Ptr{t_timeout},), time)
end

function timeout_get_process_cpu_time(time)
    ccall((:timeout_get_process_cpu_time, hil_sdk), t_error, (Ptr{t_timeout},), time)
end

function timeout_get_milliseconds(milliseconds, timeout)
    ccall((:timeout_get_milliseconds, hil_sdk), t_error, (Ptr{t_int32}, Ptr{t_timeout}), milliseconds, timeout)
end

const t_ubyte = Cuchar

const t_uint8 = t_ubyte

@cenum var"##Ctag#621"::UInt32 begin
    _false = 0
    _true = 1
end

const t_char = Cchar

const t_byte = Int8

const t_short = Cshort

const t_single = Cfloat

const t_int8 = t_byte

const t_int16 = t_short

const t_wide_char = Cwchar_t

const t_utf8_char = Cchar

const t_utf16_char = t_uint16

const t_utf32_char = Cwchar_t

const t_uint64 = t_ulong

const MAX_CARD_TYPE_LENGTH = 64

const hil_topen = hil_open

const hil_set_card_specific_toptions = hil_set_card_specific_options

const hil_get_tstring_property = hil_get_string_property

const hil_set_tstring_property = hil_set_string_property

# Skipping MacroDefinition: EXTERN extern

# Skipping MacroDefinition: EXPORT __attribute__ ( ( visibility ( "default" ) ) )

#const _utimbuf = utimbuf

const O_BINARY = 0

const O_LARGEFILE = 0

#const EFPOS = ENOTRECOVERABLE

const SH_DENYRW = 0x10

const SH_DENYWR = 0x20

const SH_DENYRD = 0x30

const SH_DENYNO = 0x40

#const _chdir = chdir

const _read = read

const _write = write

const _close = close

#const _unlink = unlink

const _stat = stat

#const _fstat = fstat

#const _getcwd = getcwd

#const _MAX_PATH = PATH_MAX

#const _MAX_FNAME = NAME_MAX

const _O_BINARY = O_BINARY

const _O_TEXT = 0

#const _O_CREAT = O_CREAT

#const _O_TRUNC = O_TRUNC

#const _O_RDWR = O_RDWR

#const _O_RDONLY = O_RDONLY

#const _O_WRONLY = O_WRONLY

const _O_SEQUENTIAL = 0

#const _S_IREAD = S_IREAD

#const _S_IWRITE = S_IWRITE

#const _S_IEXEC = S_IEXEC

#const _SH_COMPAT = SH_COMPAT

const _SH_DENYNO = SH_DENYNO

const _SH_DENYWR = SH_DENYWR

const _SH_DENYRW = SH_DENYRW

const std_topen = std_wopen

const stdfile_topen = stdfile_open

const stdfile_tgets = stdfile_gets

#const stdfile_tprintf = stdfile_printf

#const stdout_tprintf = stdout_printf

#const stdfile_tscanf = stdfile_scanf

# Skipping MacroDefinition: INLINE static __inline__

const msg_get_current_locale = msg_get_current_localeA

const msg_set_current_locale = msg_set_current_localeA

const msg_read_error_message = msg_read_error_messageA

const msg_get_error_message = msg_get_error_messageA

#const QSCHED_FIFO = SCHED_FIFO

#const QSCHED_RR = SCHED_RR

#const QSCHED_OTHER = SCHED_OTHER

#const SIGBREAK = SIGTERM

#const QSIG_BLOCK = SIG_BLOCK

#const QSIG_UNBLOCK = SIG_UNBLOCK

#const QSIG_SETMASK = SIG_SETMASK

const MAX_STRING_LENGTH = ~(Cuint(0)) >> 1

#const _tstring_format = string_format

const QTHREAD_STACK_MIN = 8192

#const QTHREAD_INHERIT_SCHED = PTHREAD_INHERIT_SCHED

#const QTHREAD_EXPLICIT_SCHED = PTHREAD_EXPLICIT_SCHED

#const QTHREAD_CANCELED = PTHREAD_CANCELED

const INVALID_QTHREAD = qthread_t(-1)

const INVALID_QTHREAD_ID = qthread_id_t(-1)

#const QTHREAD_ONCE_INIT = PTHREAD_ONCE_INIT

#const QTHREAD_CANCEL_ENABLE = PTHREAD_CANCEL_ENABLE

#const QTHREAD_CANCEL_DISABLE = PTHREAD_CANCEL_DISABLE

const MAX_TIMEOUT = 9.223372036854776e18

# exports
const PREFIXES = ["CX", "clang_"]
for name in names(@__MODULE__; all=true), prefix in PREFIXES
    if startswith(string(name), prefix)
        @eval export $name
    end
end

end # module
