//! Backup domain power, reset, and clock control
//!
//! The Backup domain is powered, clocked, and reset separately from
//! the rest of the chip; it can continue operating as long as
//! VBat is available. Therefore it is managed separately, when the chip
//! boots the backup power domain may still be configured and resetting
//! it would defeat the purpose.

use crate::stm32::{PWR, RCC};
use cortex_m::interrupt;

pub struct Backup {
    pub(crate) rb: PWR
}

impl Backup {
    pub(crate) bdcr(&mut self) -> () {
        unsafe {
            &(*RCC::ptr()).bdcr
        }
    }
    
}

