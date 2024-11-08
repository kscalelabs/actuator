use crate::types::MotorType;
use lazy_static::lazy_static;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MotorConfig {
    pub p_min: f32,
    pub p_max: f32,
    pub v_min: f32,
    pub v_max: f32,
    pub kp_min: f32,
    pub kp_max: f32,
    pub kd_min: f32,
    pub kd_max: f32,
    pub t_min: f32,
    pub t_max: f32,
    pub zero_on_init: bool,
    pub can_timeout_command: u16,
    pub can_timeout_factor: f32,
}

lazy_static! {
    pub static ref ROBSTRIDE_CONFIGS: HashMap<MotorType, MotorConfig> = {
        let mut m = HashMap::new();
        m.insert(
            MotorType::Type00,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -33.0,
                v_max: 33.0,
                kp_min: 0.0,
                kp_max: 500.0,
                kd_min: 0.0,
                kd_max: 5.0,
                t_min: -14.0,
                t_max: 14.0,
                zero_on_init: false,
                can_timeout_command: 0x200b, // Unchecked
                can_timeout_factor: 12000.0, // Unchecked
            },
        );
        m.insert(
            MotorType::Type01,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -44.0,
                v_max: 44.0,
                kp_min: 0.0,
                kp_max: 500.0,
                kd_min: 0.0,
                kd_max: 5.0,
                t_min: -12.0,
                t_max: 12.0,
                zero_on_init: true, // Single encoder motor.
                can_timeout_command: 0x200c,
                can_timeout_factor: 12000.0,
            },
        );
        m.insert(
            MotorType::Type02,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -44.0,
                v_max: 44.0,
                kp_min: 0.0,
                kp_max: 500.0,
                kd_min: 0.0,
                kd_max: 5.0,
                t_min: -12.0,
                t_max: 12.0,
                zero_on_init: false,
                can_timeout_command: 0x200b, // Unchecked
                can_timeout_factor: 12000.0, // Unchecked
            },
        );
        m.insert(
            MotorType::Type03,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -20.0,
                v_max: 20.0,
                kp_min: 0.0,
                kp_max: 5000.0,
                kd_min: 0.0,
                kd_max: 100.0,
                t_min: -60.0,
                t_max: 60.0,
                zero_on_init: false,
                can_timeout_command: 0x200b,
                can_timeout_factor: 6000.0,
            },
        );
        m.insert(
            MotorType::Type04,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -15.0,
                v_max: 15.0,
                kp_min: 0.0,
                kp_max: 5000.0,
                kd_min: 0.0,
                kd_max: 100.0,
                t_min: -120.0,
                t_max: 120.0,
                zero_on_init: false,
                can_timeout_command: 0x200b,
                can_timeout_factor: 12000.0,
            },
        );
        m
    };
}
