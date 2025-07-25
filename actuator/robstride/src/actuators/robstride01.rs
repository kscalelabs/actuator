use crate::actuator_types::{
    ActuatorParameter, EnableCommand, ParameterMetadata, ParameterType, TxCommand,
};
use crate::{
    actuator::{denormalize_value, normalize_value, TypedCommandData, TypedFeedbackData},
    Actuator, ActuatorMeasurementLimits, ActuatorType, Command, CommandData, CommunicationType,
    ControlCommand, FeedbackFrame, ObtainIDCommand, ParaStrInfo, ReadCommand, SetIDCommand,
    SetZeroCommand, StopCommand, WriteCommand,
};
use async_trait::async_trait;
use tracing::*;
use eyre::{Result, WrapErr};
use std::f32::consts::PI;
use tokio::sync::mpsc;

const LIMITS: ActuatorMeasurementLimits = ActuatorMeasurementLimits {
    min_angle: -4.0 * PI,
    max_angle: 4.0 * PI,
    min_velocity: -44.0,
    max_velocity: 44.0,
    min_torque: -17.0,
    max_torque: 17.0,
    min_kp: 0.0,
    max_kp: 500.0,
    min_kd: 0.0,
    max_kd: 5.0,
};

#[derive(Debug, Clone, Default)]
pub struct RobStride01Command {
    pub target_angle_rad: f32,     // Radians
    pub target_velocity_rads: f32, // Radians per second
    pub kp: f32,                   // Position gain
    pub kd: f32,                   // Velocity gain
    pub torque_nm: f32,            // Newton-meters
}

#[derive(Debug, Clone, Copy)]
pub struct RobStride01Feedback {
    pub angle_rad: f32,
    pub velocity_rads: f32,
    pub torque_nm: f32,
}

impl TypedCommandData for RobStride01Command {
    fn to_control_command(&self) -> ControlCommand {
        ControlCommand {
            target_angle: normalize_value(
                self.target_angle_rad,
                LIMITS.min_angle,
                LIMITS.max_angle,
                -100.0,
                100.0,
            ),
            target_velocity: normalize_value(
                self.target_velocity_rads,
                LIMITS.min_velocity,
                LIMITS.max_velocity,
                -100.0,
                100.0,
            ),
            kp: normalize_value(self.kp, LIMITS.min_kp, LIMITS.max_kp, 0.0, 100.0),
            kd: normalize_value(self.kd, LIMITS.min_kd, LIMITS.max_kd, 0.0, 100.0),
            torque: normalize_value(
                self.torque_nm,
                LIMITS.min_torque,
                LIMITS.max_torque,
                -100.0,
                100.0,
            ),
        }
    }

    fn from_control_command(cmd: ControlCommand) -> Self {
        Self {
            target_angle_rad: denormalize_value(
                cmd.target_angle,
                -100.0,
                100.0,
                LIMITS.min_angle,
                LIMITS.max_angle,
            ),
            target_velocity_rads: denormalize_value(
                cmd.target_velocity,
                -100.0,
                100.0,
                LIMITS.min_velocity,
                LIMITS.max_velocity,
            ),
            kp: denormalize_value(cmd.kp, 0.0, 100.0, LIMITS.min_kp, LIMITS.max_kp),
            kd: denormalize_value(cmd.kd, 0.0, 100.0, LIMITS.min_kd, LIMITS.max_kd),
            torque_nm: denormalize_value(
                cmd.torque,
                -100.0,
                100.0,
                LIMITS.min_torque,
                LIMITS.max_torque,
            ),
        }
    }
}

impl TypedFeedbackData for RobStride01Feedback {
    fn from_feedback_frame(frame: FeedbackFrame) -> Self {
        Self {
            angle_rad: normalize_value(
                frame.angle,
                -100.0,
                100.0,
                LIMITS.min_angle,
                LIMITS.max_angle,
            ),
            velocity_rads: normalize_value(
                frame.velocity,
                -100.0,
                100.0,
                LIMITS.min_velocity,
                LIMITS.max_velocity,
            ),
            torque_nm: normalize_value(
                frame.torque,
                -100.0,
                100.0,
                LIMITS.min_torque,
                LIMITS.max_torque,
            ),
        }
    }

    fn angle_rad(&self) -> f32 {
        self.angle_rad
    }

    fn velocity_rads(&self) -> f32 {
        self.velocity_rads
    }

    fn torque_nm(&self) -> f32 {
        self.torque_nm
    }
}

#[derive(Debug, Clone)]
pub struct RobStride01 {
    pub id: u8,
    pub host_id: u8,
    pub tx: mpsc::Sender<TxCommand>,
}

impl RobStride01 {
    pub fn new(id: u8, host_id: u8, tx: mpsc::Sender<TxCommand>) -> Self {
        Self { id, host_id, tx }
    }
}

#[async_trait]
impl Actuator for RobStride01 {
    fn id(&self) -> u8 {
        self.id
    }

    fn actuator_type(&self) -> ActuatorType {
        ActuatorType::RobStride01
    }

    async fn enable(&self) -> Result<()> {
        let cmd = EnableCommand {
            host_id: self.host_id,
        }
        .to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send enable command")?;
        Ok(())
    }

    async fn disable(&self, clear_fault: bool) -> Result<()> {
        let cmd = StopCommand {
            host_id: self.host_id,
            clear_fault,
        }
        .to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send disable command")?;
        Ok(())
    }

    async fn set_id(&mut self, id: u8) -> Result<()> {
        let cmd = SetIDCommand {
            host_id: self.host_id,
            new_id: id,
        }
        .to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send set_id command")?;
        self.id = id;
        Ok(())
    }

    async fn get_uuid(&self) -> Result<()> {
        let cmd = ObtainIDCommand {
            host_id: self.host_id,
        }
        .to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send get_uuid command")?;
        Ok(())
    }

    async fn control(&self, cmd: ControlCommand) -> Result<()> {
        let cmd = cmd.to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send control command")?;
        Ok(())
    }

    #[tracing::instrument]
    async fn get_feedback(&self) -> Result<()> {
        let cmd = Command {
            data: [0; 8],
            can_id: self.id,
            data_2: 0,
            communication_type: CommunicationType::Feedback,
        }
        .to_can_packet();

        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send get_feedback command")?;
        Ok(())
    }

    async fn write_parameter(&self, cmd: WriteCommand) -> Result<()> {
        let cmd = cmd.to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send write_parameter command")?;
        Ok(())
    }

    async fn read_parameter(&self, param_index: u16) -> Result<()> {
        let cmd = ReadCommand {
            host_id: self.host_id,
            parameter_index: param_index,
            data: 0,
            read_status: false,
        }
        .to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send read_parameter command")?;
        Ok(())
    }

    async fn get_parameter_string_info(&self) -> Result<()> {
        let cmd = ParaStrInfo {
            host_id: self.host_id,
        }
        .to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send get_parameter_string_info command")?;
        Ok(())
    }

    async fn set_zero(&self) -> Result<()> {
        let cmd = SetZeroCommand {
            host_id: self.host_id,
        }
        .to_can_packet(self.id);
        self.tx
            .send(TxCommand::Send {
                id: cmd.0,
                data: cmd.1,
            })
            .await
            .wrap_err("failed to send set_zero command")?;
        Ok(())
    }

    async fn set_max_torque(&self, torque: f32) -> Result<()> {
        let param = RobStride01Parameter::LimitTorque;
        let cmd = WriteCommand {
            host_id: self.host_id,
            parameter_index: param.metadata().index,
            data: torque,
        };
        self.write_parameter(cmd).await
    }

    async fn set_max_velocity(&self, velocity: f32) -> Result<()> {
        let param = RobStride01Parameter::LimitSpd;
        let cmd = WriteCommand {
            host_id: self.host_id,
            parameter_index: param.metadata().index,
            data: velocity,
        };
        self.write_parameter(cmd).await
    }

    async fn set_max_current(&self, current: f32) -> Result<()> {
        let param = RobStride01Parameter::LimitCur;
        let cmd = WriteCommand {
            host_id: self.host_id,
            parameter_index: param.metadata().index,
            data: current,
        };
        self.write_parameter(cmd).await
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RobStride01Parameter {
    RunMode,     // 0x7005 - Operation control mode (0-3)
    IqRef,       // 0x7006 - Current mode Iq command (-90A to 90A)
    SpdRef,      // 0x700A - Speed mode command (-15 to 15 rad/s)
    LimitTorque, // 0x700B - Torque limitation (0-120 Nm)
    CurKp,       // 0x7010 - Current Kp (default 0.05)
    CurKi,       // 0x7011 - Current Ki (default 0.05)
    CurFitGain,  // 0x7014 - Current filter coefficient (0-1.0, default 0.06)
    Ref,         // 0x7016 - Position mode angle command (rad)
    LimitSpd,    // 0x7017 - Position mode speed limit (0-15 rad/s)
    LimitCur,    // 0x7018 - Speed/position mode current limit (0-90A)
    MechPos,     // 0x7019 - Load end mechanical angle (rad)
    Iqf,         // 0x701A - Iq filter values (-90A to 90A)
    MechVel,     // 0x701B - Load end speed (-15 to 15 rad/s)
    VBus,        // 0x701C - Bus voltage (V)
    LocKp,       // 0x701E - Position Kp (default 30)
    SpdKp,       // 0x701F - Speed Kp (default 5)
    SpdKi,       // 0x7020 - Speed Ki (default 0.005)
    SpdFiltGain, // 0x7021 - Speed filter gain (default 0.1)
    Unknown,
}

impl ActuatorParameter for RobStride01Parameter {
    fn metadata(&self) -> ParameterMetadata {
        match self {
            Self::RunMode => ParameterMetadata {
                index: 0x7005,
                name: String::from("Run Mode"),
                param_type: ParameterType::Uint8,
                units: String::from("mode"),
                min_value: Some(0.0),
                max_value: Some(3.0),
            },
            Self::IqRef => ParameterMetadata {
                index: 0x7006,
                name: String::from("Iq Reference"),
                param_type: ParameterType::Float,
                units: String::from("A"),
                min_value: Some(-23.0),
                max_value: Some(23.0),
            },
            Self::SpdRef => ParameterMetadata {
                index: 0x700A,
                name: String::from("Speed Reference"),
                param_type: ParameterType::Float,
                units: String::from("rad/s"),
                min_value: Some(-44.0),
                max_value: Some(44.0),
            },
            Self::LimitTorque => ParameterMetadata {
                index: 0x700B,
                name: String::from("Torque Limit"),
                param_type: ParameterType::Float,
                units: String::from("Nm"),
                min_value: Some(0.0),
                max_value: Some(17.0),
            },
            Self::CurKp => ParameterMetadata {
                index: 0x7010,
                name: String::from("Current Kp"),
                param_type: ParameterType::Float,
                units: String::from(""),
                min_value: Some(0.0),
                max_value: None,
            },
            Self::CurKi => ParameterMetadata {
                index: 0x7011,
                name: String::from("Current Ki"),
                param_type: ParameterType::Float,
                units: String::from(""),
                min_value: Some(0.0),
                max_value: None,
            },
            Self::CurFitGain => ParameterMetadata {
                index: 0x7014,
                name: String::from("Current Filter Gain"),
                param_type: ParameterType::Float,
                units: String::from(""),
                min_value: Some(0.0),
                max_value: Some(1.0),
            },
            Self::Ref => ParameterMetadata {
                index: 0x7016,
                name: String::from("Position Reference"),
                param_type: ParameterType::Float,
                units: String::from("rad"),
                min_value: None,
                max_value: None,
            },
            Self::LimitSpd => ParameterMetadata {
                index: 0x7017,
                name: String::from("Speed Limit"),
                param_type: ParameterType::Float,
                units: String::from("rad/s"),
                min_value: Some(0.0),
                max_value: Some(44.0),
            },
            Self::LimitCur => ParameterMetadata {
                index: 0x7018,
                name: String::from("Current Limit"),
                param_type: ParameterType::Float,
                units: String::from("A"),
                min_value: Some(0.0),
                max_value: Some(23.0),
            },
            Self::MechPos => ParameterMetadata {
                index: 0x7019,
                name: String::from("Mechanical Position"),
                param_type: ParameterType::Float,
                units: String::from("rad"),
                min_value: None,
                max_value: None,
            },
            Self::Iqf => ParameterMetadata {
                index: 0x701A,
                name: String::from("Iq Filter Value"),
                param_type: ParameterType::Float,
                units: String::from("A"),
                min_value: Some(-23.0),
                max_value: Some(23.0),
            },
            Self::MechVel => ParameterMetadata {
                index: 0x701B,
                name: String::from("Mechanical Velocity"),
                param_type: ParameterType::Float,
                units: String::from("rad/s"),
                min_value: Some(-44.0),
                max_value: Some(44.0),
            },
            Self::VBus => ParameterMetadata {
                index: 0x701C,
                name: String::from("Bus Voltage"),
                param_type: ParameterType::Float,
                units: String::from("V"),
                min_value: None,
                max_value: None,
            },
            Self::LocKp => ParameterMetadata {
                index: 0x701E,
                name: String::from("Position Kp"),
                param_type: ParameterType::Float,
                units: String::from(""),
                min_value: Some(0.0),
                max_value: None,
            },
            Self::SpdKp => ParameterMetadata {
                index: 0x701F,
                name: String::from("Speed Kp"),
                param_type: ParameterType::Float,
                units: String::from(""),
                min_value: Some(0.0),
                max_value: None,
            },
            Self::SpdKi => ParameterMetadata {
                index: 0x7020,
                name: String::from("Speed Ki"),
                param_type: ParameterType::Float,
                units: String::from(""),
                min_value: Some(0.0),
                max_value: None,
            },
            Self::Unknown => ParameterMetadata {
                index: 0x0000,
                name: String::from("Unknown"),
                param_type: ParameterType::Uint16,
                units: String::from(""),
                min_value: None,
                max_value: None,
            },
            Self::SpdFiltGain => ParameterMetadata {
                index: 0x7021,
                name: String::from("Speed Filter Gain"),
                param_type: ParameterType::Float,
                units: String::from(""),
                min_value: Some(0.0),
                max_value: Some(1.0),
            },
        }
    }
}

impl RobStride01Parameter {
    pub fn iter() -> impl Iterator<Item = RobStride01Parameter> {
        use RobStride01Parameter::*;
        [
            RunMode,
            IqRef,
            SpdRef,
            LimitTorque,
            CurKp,
            CurKi,
            CurFitGain,
            Ref,
            LimitSpd,
            LimitCur,
            MechPos,
            Iqf,
            MechVel,
            VBus,
            LocKp,
            SpdKp,
            SpdKi,
            SpdFiltGain,
        ]
        .iter()
        .cloned()
    }

    pub fn from_index(index: u16) -> Option<Self> {
        match index {
            0x7005 => Some(Self::RunMode),
            0x7006 => Some(Self::IqRef),
            0x700A => Some(Self::SpdRef),
            0x700B => Some(Self::LimitTorque),
            0x7010 => Some(Self::CurKp),
            0x7011 => Some(Self::CurKi),
            0x7014 => Some(Self::CurFitGain),
            0x7016 => Some(Self::Ref),
            0x7017 => Some(Self::LimitSpd),
            0x7018 => Some(Self::LimitCur),
            0x7019 => Some(Self::MechPos),
            0x701A => Some(Self::Iqf),
            0x701B => Some(Self::MechVel),
            0x701C => Some(Self::VBus),
            0x701E => Some(Self::LocKp),
            0x701F => Some(Self::SpdKp),
            0x7020 => Some(Self::SpdKi),
            0x7021 => Some(Self::SpdFiltGain),

            _ => None,
        }
    }
}
