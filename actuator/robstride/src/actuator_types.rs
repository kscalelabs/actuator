use num_derive::{FromPrimitive, ToPrimitive};

#[derive(Debug, Clone, Copy, FromPrimitive)]
pub enum CommunicationType {
    // Obtain Device ID
    ObtainID = 0,
    // Motor Control Command
    Control = 1,
    // Motor Feedback Data
    Feedback = 2,
    // Motor Enable Operation
    Enable = 3,
    // Motor Stop Running
    Stop = 4,
    // Set Mechanical Zero
    SetZero = 6,
    // Set CAN ID
    SetID = 7,
    // Single Parameter Read
    Read = 17,
    // Single Parameter Write
    Write = 18,
    // Parameter String Info
    ParaStrInfo = 19,
    // Fault Feedback (04)
    Fault = 21,
}

#[derive(Debug, Clone, Copy, FromPrimitive)]
pub enum CanComMode {
    AnnounceDevId = 0,
    MotorCtrl = 1,
    MotorFeedback = 2,
    MotorIn = 3,
    MotorReset = 4,
    MotorCali = 5,
    MotorZero = 6,
    MotorId = 7,
    ParaWrite = 8,
    ParaRead = 9,
    ParaUpdate = 10,
    OtaStart = 11,
    OtaInfo = 12,
    OtaIng = 13,
    OtaEnd = 14,
    CaliIng = 15,
    CaliRst = 16,
    SdoRead = 17,
    SdoWrite = 18,
    // Fault Feedback (04)
    ParaStrInfo = 19,
    MotorBrake = 20,
    FaultWarn = 21,
    ModeTotal = 22,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Frame {
    ObtainID(ObtainIDCommand),
    Control(ControlCommand),
    Feedback(FeedbackFrame),
    Read(ReadCommand),
    Write(WriteCommand),
    Fault(FaultFeedback),
    Enable(EnableCommand),
    Stop(StopCommand),
    SetZero(SetZeroCommand),
    SetID(SetIDCommand),
}

#[derive(Debug, Clone, Copy)]
pub enum ParameterType {
    Uint8,
    Uint16,
    Uint32,
    Int8,
    Int16,
    Int32,
    Float,
    String,
}

#[derive(Debug, Clone)]
pub struct ParameterMetadata {
    pub index: u16,
    pub name: String,
    pub param_type: ParameterType,
    pub units: String,
    pub min_value: Option<f32>,
    pub max_value: Option<f32>,
}

// Base trait for all parameter types
pub trait ActuatorParameter {
    fn metadata(&self) -> ParameterMetadata;
}

#[derive(Debug, Clone, Copy, FromPrimitive, PartialEq)]
pub enum ActuatorType {
    RobStride00 = 0,
    RobStride01 = 1,
    RobStride02 = 2,
    RobStride03 = 3,
    RobStride04 = 4,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ObtainIDCommand {
    pub host_id: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ControlCommand {
    pub target_angle: f32,    // -100.0 to 100.0
    pub target_velocity: f32, // -100.0 to 100.0
    pub kp: f32,              // 0.0 to 100.0
    pub kd: f32,              // 0.0 to 100.0
    pub torque: f32,          // -100.0 to 100.0
}

#[derive(Debug, Clone, PartialEq)]
pub struct FeedbackFrame {
    pub angle: f32,                    // -100.0 to 100.0
    pub velocity: f32,                 // -100.0 to 100.0
    pub torque: f32,                   // -100.0 to 100.0
    pub temperature: f32,              // Celsius
    pub fault_uncalibrated: bool,      // Bit21
    pub fault_hall_encoding: bool,     // Bit20
    pub fault_magnetic_encoding: bool, // Bit19
    pub fault_over_temperature: bool,  // Bit18
    pub fault_overcurrent: bool,       // Bit17
    pub fault_undervoltage: bool,      // Bit16
    pub mode: MotorMode,               // Bits 22-23
    pub motor_id: u8,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive, PartialEq)]
pub enum MotorMode {
    Reset = 0,       // Reset mode
    Calibration = 1, // Cali mode
    Run = 2,         // Motor mode
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaultFrame {
    pub fault_code: u16,
    pub fault_message: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EnableCommand {
    pub host_id: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StopCommand {
    pub host_id: u8,
    pub clear_fault: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SetZeroCommand {
    pub host_id: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SetIDCommand {
    pub host_id: u8,
    pub new_id: u8,
}

pub struct ParaStrInfo {
    pub host_id: u8,
}

#[derive(Clone, PartialEq)]
pub struct ReadCommand {
    pub host_id: u8,
    pub parameter_index: u16,
    pub data: u32,
    pub read_status: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WriteCommand {
    pub host_id: u8,
    pub parameter_index: u16,
    pub data: f32,
}
#[derive(Debug, Clone, PartialEq)]
pub struct FaultFeedback {
    // Fault values (Byte 0-3)
    pub phase_a_overcurrent: bool,
    pub overload_fault: bool,
    pub encoder_not_calibrated: bool,
    pub phase_c_overcurrent: bool,
    pub phase_b_overcurrent: bool,
    pub overvoltage_fault: bool,
    pub undervoltage_fault: bool,
    pub driver_chip_failure: bool,
    pub motor_over_temp_fault: bool,

    // Warning values (Byte 4-7)
    pub motor_over_temp_warning: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TxCommand {
    Send { id: u32, data: Vec<u8> },
}

pub struct ActuatorMeasurementLimits {
    pub min_angle: f32,
    pub max_angle: f32,
    pub min_velocity: f32,
    pub max_velocity: f32,
    pub min_torque: f32,
    pub max_torque: f32,
    pub min_kp: f32,
    pub max_kp: f32,
    pub min_kd: f32,
    pub max_kd: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ActuatorConfiguration {
    pub actuator_type: ActuatorType,
    pub max_angle_change: Option<f32>,
    pub max_velocity: Option<f32>,
}

impl Default for ActuatorConfiguration {
    fn default() -> Self {
        ActuatorConfiguration {
            actuator_type: ActuatorType::RobStride04,
            max_angle_change: None,
            max_velocity: None,
        }
    }
}
