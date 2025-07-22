use crate::actuator_types::*;
use crate::robstride04::RobStride04Parameter;
use async_trait::async_trait;
use eyre::Result;
use num_traits::{FromPrimitive, ToPrimitive};
use serde::{Deserialize, Serialize};

#[tracing::instrument(ret(Debug))]
pub fn normalize_value(
    value: f32,
    min: f32,
    max: f32,
    out_range_min: f32,
    out_range_max: f32,
) -> f32 {
    let range = max - min;
    ((value - min) * ((out_range_max - out_range_min) / range) + out_range_min)
        .clamp(out_range_min, out_range_max)
}


#[tracing::instrument(ret(Debug))]
pub fn denormalize_value(
    normalized: f32,
    min: f32,
    max: f32,
    in_range_min: f32,
    in_range_max: f32,
) -> f32 {
    let range = in_range_max - in_range_min;
    ((normalized - in_range_min) * range / (in_range_max - in_range_min) + min).clamp(min, max)
}

pub trait TypedCommandData: Send + Sync {
    fn to_control_command(&self) -> ControlCommand;
    fn from_control_command(cmd: ControlCommand) -> Self
    where
        Self: Sized;
}

pub trait TypedFeedbackData: Send + Sync {
    fn from_feedback_frame(frame: FeedbackFrame) -> Self
    where
        Self: Sized;
    fn angle_rad(&self) -> f32;
    fn velocity_rads(&self) -> f32;
    fn torque_nm(&self) -> f32;
}

#[async_trait]
pub trait Actuator: Send + Sync + std::fmt::Debug {
    async fn enable(&self) -> Result<()>;
    async fn disable(&self, clear_fault: bool) -> Result<()>;
    async fn set_id(&mut self, id: u8) -> Result<()>;
    async fn get_uuid(&self) -> Result<()>;
    async fn control(&self, cmd: ControlCommand) -> Result<()>;
    async fn get_feedback(&self) -> Result<()>;
    async fn set_zero(&self) -> Result<()>;

    fn id(&self) -> u8;
    fn actuator_type(&self) -> ActuatorType;

    async fn write_parameter(&self, cmd: WriteCommand) -> Result<()>;
    async fn read_parameter(&self, param_index: u16) -> Result<()>;
    async fn get_parameter_string_info(&self) -> Result<()>;

    async fn set_max_torque(&self, torque: f32) -> Result<()>;
    async fn set_max_velocity(&self, velocity: f32) -> Result<()>;
    async fn set_max_current(&self, current: f32) -> Result<()>;
}

#[derive(Debug, Clone)]
pub struct Command {
    pub data: [u8; 8],
    pub can_id: u8,
    pub data_2: u16,
    pub communication_type: CommunicationType,
}

impl Command {
    pub fn new(data: [u8; 8], can_id: u8, data_2: u16, comm_type: CommunicationType) -> Self {
        Command {
            data,
            can_id,
            data_2,
            communication_type: comm_type,
        }
    }
    // Convert from CAN packet format
    pub fn from_can_packet(id: u32, mut data: Vec<u8>) -> Self {
        // Extract fields from id
        let can_id = (id & 0x7F) as u8; // First 7 bits
        let data_2 = ((id >> 8) & 0xFFFF) as u16; // Bits 8-23
        let comm_type = ((id >> 24) & 0x1F) as u8; // Bits 24-28

        // Pad data to 8 bytes if needed TODO prepend vs append
        data.resize(8, 0);
        let data: [u8; 8] = data.try_into().unwrap();

        Command {
            data,
            can_id,
            data_2,
            communication_type: CommunicationType::from_u8(comm_type)
                .unwrap_or(CommunicationType::Control),
        }
    }

    // Convert to CAN packet format
    pub fn to_can_packet(&self) -> (u32, Vec<u8>) {
        let id: u32 = (self.can_id as u32)
            | ((self.data_2 as u32) << 8)
            | ((self.communication_type as u32) << 24);

        (id, self.data.to_vec())
    }

    pub fn to_frame(&self) -> Result<Frame, String> {
        match self.communication_type {
            CommunicationType::ObtainID => {
                Ok(Frame::ObtainID(ObtainIDCommand::from_command(self.clone())))
            }
            CommunicationType::Control => {
                Ok(Frame::Control(ControlCommand::from_command(self.clone())))
            }
            CommunicationType::Feedback => {
                Ok(Frame::Feedback(FeedbackFrame::from_command(self.clone())))
            }
            CommunicationType::Read => Ok(Frame::Read(ReadCommand::from_command(self.clone()))),
            // CommunicationType::Stop => Ok(Frame::Feedback(
            //     FeedbackFrame::from_command(self.clone()),
            // )),
            // CommunicationType::Enable => Ok(Frame::Feedback(
            //     FeedbackFrame::from_command(self.clone()),
            // )),
            CommunicationType::Fault => {
                // Parse fault data from the command
                let fault_values = u32::from_le_bytes(self.data[0..4].try_into().unwrap());
                let warning_values = u32::from_le_bytes(self.data[4..8].try_into().unwrap());

                let fault_feedback = FaultFeedback {
                    phase_a_overcurrent: (fault_values & (1 << 13)) != 0,
                    overload_fault: (fault_values & (1 << 14)) != 0,
                    encoder_not_calibrated: (fault_values & (1 << 7)) != 0,
                    phase_c_overcurrent: (fault_values & (1 << 12)) != 0,
                    phase_b_overcurrent: (fault_values & (1 << 11)) != 0,
                    overvoltage_fault: (fault_values & (1 << 3)) != 0,
                    undervoltage_fault: (fault_values & (1 << 2)) != 0,
                    driver_chip_failure: (fault_values & (1 << 1)) != 0,
                    motor_over_temp_fault: (fault_values & 1) != 0,
                    motor_over_temp_warning: (warning_values & 1) != 0,
                };

                Ok(Frame::Fault(fault_feedback))
            }
            _ => Err("Invalid communication type".to_string()),
        }
    }
}

impl Serialize for Command {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let (id, data) = self.to_can_packet();
        (id, data).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Command {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (id, data): (u32, Vec<u8>) = Deserialize::deserialize(deserializer)?;
        Ok(Command::from_can_packet(id, data))
    }
}

pub trait CommandData {
    fn command_type(&self) -> CommunicationType;
    fn from_command(cmd: Command) -> Self;
    fn to_command(&self, can_id: u8) -> Command;
    fn to_can_packet(&self, can_id: u8) -> (u32, Vec<u8>) {
        let cmd = self.to_command(can_id);
        cmd.to_can_packet()
    }
}

impl CommandData for ObtainIDCommand {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::ObtainID
    }
    fn from_command(cmd: Command) -> Self {
        let host_id = cmd.data_2 as u8;
        ObtainIDCommand { host_id }
    }
    fn to_command(&self, can_id: u8) -> Command {
        Command::new(
            [0; 8],
            can_id,
            self.host_id as u16,
            CommunicationType::ObtainID,
        )
    }
}

impl CommandData for ControlCommand {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::Control
    }
    fn from_command(cmd: Command) -> Self {
        // Convert pairs of bytes to u16 using from_le_bytes
        let angle_raw = u16::from_be_bytes(cmd.data[0..2].try_into().unwrap());
        let velocity_raw = u16::from_be_bytes(cmd.data[2..4].try_into().unwrap());
        let kp_raw = u16::from_be_bytes(cmd.data[4..6].try_into().unwrap());
        let kd_raw = u16::from_be_bytes(cmd.data[6..8].try_into().unwrap());

        // Convert to final values
        let target_angle = denormalize_value(angle_raw as f32, 0.0, 65535.0, -100.0, 100.0);
        let target_velocity = denormalize_value(velocity_raw as f32, 0.0, 65535.0, -100.0, 100.0);
        let kp = denormalize_value(kp_raw as f32, 0.0, 65535.0, 0.0, 100.0);
        let kd = denormalize_value(kd_raw as f32, 0.0, 65535.0, 0.0, 100.0);

        let torque = (cmd.data_2 as f32) * 240.0 / 65535.0 - 120.0;

        ControlCommand {
            target_angle,
            target_velocity,
            kp,
            kd,
            torque,
        }
    }

    fn to_command(&self, can_id: u8) -> Command {
        let mut data = [0u8; 8];

        // Use the new normalization function with explicit limits
        let angle_normalized =
            normalize_value(self.target_angle, -100.0, 100.0, 0.0, 65535.0) as u16;
        let velocity_normalized =
            normalize_value(self.target_velocity, -100.0, 100.0, 0.0, 65535.0) as u16;
        let kp_normalized = normalize_value(self.kp, 0.0, 100.0, 0.0, 65535.0) as u16;
        let kd_normalized = normalize_value(self.kd, 0.0, 100.0, 0.0, 65535.0) as u16;
        let torque_normalized = normalize_value(self.torque, -100.0, 100.0, 0.0, 65535.0) as u16;

        // Convert to bytes using to_be_bytes and copy to data array
        data[0..2].copy_from_slice(&angle_normalized.to_be_bytes());
        data[2..4].copy_from_slice(&velocity_normalized.to_be_bytes());
        data[4..6].copy_from_slice(&kp_normalized.to_be_bytes());
        data[6..8].copy_from_slice(&kd_normalized.to_be_bytes());

        // debug!("angle: {}, data: {:x?}", self.target_angle, &data[0..2]);

        Command::new(
            data,
            can_id,
            torque_normalized as u16,
            CommunicationType::Control,
        )
    }
}

impl CommandData for FeedbackFrame {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::Feedback
    }

    fn from_command(cmd: Command) -> Self {
        let angle_raw = u16::from_be_bytes(cmd.data[0..2].try_into().unwrap());
        let velocity_raw = u16::from_be_bytes(cmd.data[2..4].try_into().unwrap());
        let torque_raw = u16::from_be_bytes(cmd.data[4..6].try_into().unwrap());
        let temp_raw = u16::from_be_bytes(cmd.data[6..8].try_into().unwrap());

        let angle = normalize_value(angle_raw as f32, 0.0, 65535.0, -100.0, 100.0);
        let velocity = normalize_value(velocity_raw as f32, 0.0, 65535.0, -100.0, 100.0);
        let torque = normalize_value(torque_raw as f32, 0.0, 65535.0, -100.0, 100.0);
        let temperature = (temp_raw as f32) / 10.0;

        // Parse data_2 status bits
        let motor_id: u8 = ((cmd.data_2) & 0xFF) as u8;
        let fault_bits = (cmd.data_2 >> 8) & 0x3F; // Bits 16-21
        let mode_bits = (cmd.data_2 >> 14) & 0x03; // Bits 22-23

        let mode = match mode_bits {
            0 => MotorMode::Reset,
            1 => MotorMode::Calibration,
            2 => MotorMode::Run,
            _ => MotorMode::Reset, // Default to Reset for unknown values
        };

        FeedbackFrame {
            angle,
            velocity,
            torque,
            temperature,
            motor_id,
            fault_uncalibrated: (fault_bits & (1 << 5)) != 0,
            fault_hall_encoding: (fault_bits & (1 << 4)) != 0,
            fault_magnetic_encoding: (fault_bits & (1 << 3)) != 0,
            fault_over_temperature: (fault_bits & (1 << 2)) != 0,
            fault_overcurrent: (fault_bits & (1 << 1)) != 0,
            fault_undervoltage: (fault_bits & (1 << 0)) != 0,
            mode,
        }
    }

    fn to_command(&self, can_id: u8) -> Command {
        let mut data = [0u8; 8];

        // Calculate normalized values
        let angle_normalized = normalize_value(self.angle, -100.0, 100.0, 0.0, 65535.0) as u16;
        let velocity_normalized =
            normalize_value(self.velocity, -100.0, 100.0, 0.0, 65535.0) as u16;
        let torque_normalized = normalize_value(self.torque, -100.0, 100.0, 0.0, 65535.0) as u16;
        let temperature_normalized = (self.temperature * 10.0) as u16;

        // Convert to bytes using to_be_bytes and copy to data array
        data[0..2].copy_from_slice(&angle_normalized.to_be_bytes());
        data[2..4].copy_from_slice(&velocity_normalized.to_be_bytes());
        data[4..6].copy_from_slice(&torque_normalized.to_be_bytes());
        data[6..8].copy_from_slice(&temperature_normalized.to_be_bytes());

        let data_2: u16 = (self.motor_id as u16)
            | ((self.fault_uncalibrated as u16) << 5)
            | ((self.fault_hall_encoding as u16) << 4)
            | ((self.fault_magnetic_encoding as u16) << 3)
            | ((self.fault_over_temperature as u16) << 2)
            | ((self.fault_overcurrent as u16) << 1)
            | (self.fault_undervoltage as u16)
            | (self.mode.to_u16().unwrap() << 8);

        Command::new(data, can_id, data_2, CommunicationType::Feedback)
    }
}

impl CommandData for StopCommand {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::Control
    }
    fn from_command(cmd: Command) -> Self {
        StopCommand {
            host_id: cmd.data_2 as u8,
            clear_fault: cmd.data[0] != 0,
        }
    }
    fn to_command(&self, can_id: u8) -> Command {
        let mut data = [0u8; 8];
        data[0] = self.clear_fault as u8;
        Command::new(data, can_id, self.host_id as u16, CommunicationType::Stop)
    }
}

impl CommandData for EnableCommand {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::Enable
    }
    fn from_command(cmd: Command) -> Self {
        EnableCommand {
            host_id: cmd.data_2 as u8,
        }
    }
    fn to_command(&self, can_id: u8) -> Command {
        Command::new(
            [0; 8],
            can_id,
            self.host_id as u16,
            CommunicationType::Enable,
        )
    }
}

impl CommandData for SetIDCommand {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::SetID
    }

    fn from_command(cmd: Command) -> Self {
        SetIDCommand {
            host_id: (cmd.data_2 & 0xFF) as u8,       // Lower 8 bits
            new_id: ((cmd.data_2 >> 8) & 0xFF) as u8, // Upper 8 bits of data_2
        }
    }

    fn to_command(&self, can_id: u8) -> Command {
        Command::new(
            [0; 8],
            can_id,
            (self.host_id as u16) | ((self.new_id as u16) << 8),
            CommunicationType::SetID,
        )
    }
}

impl CommandData for WriteCommand {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::Write
    }
    fn from_command(cmd: Command) -> Self {
        WriteCommand {
            host_id: cmd.data_2 as u8,
            parameter_index: u16::from_le_bytes(cmd.data[0..=1].try_into().unwrap()),
            data: f32::from_le_bytes(cmd.data[4..=7].try_into().unwrap()),
        }
    }
    fn to_command(&self, can_id: u8) -> Command {
        let mut data = [0u8; 8];
        data[0..=1].copy_from_slice(&self.parameter_index.to_le_bytes());

        if RobStride04Parameter::from_index(self.parameter_index)
            .unwrap_or(RobStride04Parameter::Unknown)
            == RobStride04Parameter::RunMode
        {
            data[4] = self.data.to_le_bytes()[0];
        } else {
            let le_data = self.data.to_le_bytes();
            data[4..=7].copy_from_slice(&le_data);
        }
        Command::new(data, can_id, self.host_id as u16, CommunicationType::Write)
    }
}

impl std::fmt::Debug for ReadCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ReadCommand(host_id: {}, parameter: {:?}, data: {}, status: {})",
            self.host_id,
            RobStride04Parameter::from_index(self.parameter_index)
                .unwrap_or(RobStride04Parameter::Unknown),
            self.data,
            self.read_status
        )
    }
}

impl CommandData for ReadCommand {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::Read
    }
    fn from_command(cmd: Command) -> Self {
        ReadCommand {
            host_id: cmd.data_2 as u8,
            parameter_index: u16::from_le_bytes(cmd.data[0..=1].try_into().unwrap()),
            data: u32::from_le_bytes(cmd.data[4..=7].try_into().unwrap()),
            read_status: cmd.data[0] == 0,
        }
    }
    fn to_command(&self, can_id: u8) -> Command {
        let mut data = [0u8; 8];
        data[0..=1].copy_from_slice(&self.parameter_index.to_le_bytes());
        data[4..=7].copy_from_slice(&self.data.to_le_bytes());
        Command::new(data, can_id, self.host_id as u16, CommunicationType::Read)
    }
}

impl ReadCommand {
    pub fn data_as_f32(&self) -> f32 {
        // TODO: shortcut, fix
        if RobStride04Parameter::from_index(self.parameter_index)
            .unwrap_or(RobStride04Parameter::Unknown)
            == RobStride04Parameter::RunMode
        {
            let data = self.data.to_le_bytes()[0];
            return data as f32;
        }
        let le_data = u32::from_le(self.data);
        let float_value = f32::from_bits(le_data);
        float_value
    }
}

impl CommandData for ParaStrInfo {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::ParaStrInfo
    }
    fn from_command(cmd: Command) -> Self {
        ParaStrInfo {
            host_id: cmd.data_2 as u8,
        }
    }
    fn to_command(&self, can_id: u8) -> Command {
        Command::new(
            [0x95, 0x8b, 0x30, 0x02, 0x0c, 0x34, 0x38, 0x0d],
            can_id,
            self.host_id as u16,
            CommunicationType::ParaStrInfo,
        )
    }
}

impl CommandData for SetZeroCommand {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::SetZero
    }
    fn from_command(cmd: Command) -> Self {
        SetZeroCommand {
            host_id: cmd.data_2 as u8,
        }
    }
    fn to_command(&self, can_id: u8) -> Command {
        let mut data = [0u8; 8];
        data[0] = 1;
        Command::new(
            data,
            can_id,
            self.host_id as u16,
            CommunicationType::SetZero,
        )
    }
}

impl CommandData for FaultFeedback {
    fn command_type(&self) -> CommunicationType {
        CommunicationType::Fault
    }

    fn from_command(cmd: Command) -> Self {
        let fault_values = u32::from_le_bytes(cmd.data[0..4].try_into().unwrap());
        let warning_values = u32::from_le_bytes(cmd.data[4..8].try_into().unwrap());

        FaultFeedback {
            phase_a_overcurrent: (fault_values & (1 << 13)) != 0,
            overload_fault: (fault_values & (1 << 14)) != 0,
            encoder_not_calibrated: (fault_values & (1 << 7)) != 0,
            phase_c_overcurrent: (fault_values & (1 << 12)) != 0,
            phase_b_overcurrent: (fault_values & (1 << 11)) != 0,
            overvoltage_fault: (fault_values & (1 << 3)) != 0,
            undervoltage_fault: (fault_values & (1 << 2)) != 0,
            driver_chip_failure: (fault_values & (1 << 1)) != 0,
            motor_over_temp_fault: (fault_values & 1) != 0,
            motor_over_temp_warning: (warning_values & 1) != 0,
        }
    }

    fn to_command(&self, can_id: u8) -> Command {
        let mut fault_values: u32 = 0;
        let mut warning_values: u32 = 0;

        if self.phase_a_overcurrent {
            fault_values |= 1 << 13;
        }
        if self.overload_fault {
            fault_values |= 1 << 14;
        }
        if self.encoder_not_calibrated {
            fault_values |= 1 << 7;
        }
        if self.phase_c_overcurrent {
            fault_values |= 1 << 12;
        }
        if self.phase_b_overcurrent {
            fault_values |= 1 << 11;
        }
        if self.overvoltage_fault {
            fault_values |= 1 << 3;
        }
        if self.undervoltage_fault {
            fault_values |= 1 << 2;
        }
        if self.driver_chip_failure {
            fault_values |= 1 << 1;
        }
        if self.motor_over_temp_fault {
            fault_values |= 1;
        }
        if self.motor_over_temp_warning {
            warning_values |= 1;
        }

        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&fault_values.to_le_bytes());
        data[4..8].copy_from_slice(&warning_values.to_le_bytes());

        Command::new(data, can_id, 0, CommunicationType::Fault)
    }
}
