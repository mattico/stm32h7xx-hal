
use core::marker::PhantomData;
use cortex_m::interrupt;

use crate::stm32::{PWR, RCC};
use crate::rcc::rec::ResetEnable;

pub struct Backup {
    pub(crate) rb: PWR,
    #[cfg(feature = "rtc")]
    pub rtc: Rtc,
}

/// Owned ability to Reset, Enable and Disable peripheral
#[cfg(feature = "rtc")]
pub struct Rtc {
    pub(crate) _marker: PhantomData<*const ()>,
}

#[cfg(feature = "rtc")]
unsafe impl Send for Rtc {}

#[cfg(feature = "rtc")]
impl ResetEnable for Rtc {
    #[inline(always)]
    fn enable(self) -> Self {
        // unsafe: Owned exclusive access to this bitfield
        interrupt::free(|_| {
            let enr = unsafe {
                &(*RCC::ptr()).bdcr
            };
            enr.modify(|_, w| w.rtcen().set_bit());
        });
        self
    }
    #[inline(always)]
    fn disable(self) -> Self {
        // unsafe: Owned exclusive access to this bitfield
        interrupt::free(|_| {
            let enr = unsafe {
                &(*RCC::ptr()).bdcr
            };
            enr.modify(|_, w| w.rtcen().clear_bit());
        });
        self
    }
    #[inline(always)]
    fn reset(self) -> Self {
        // unsafe: Owned exclusive access to this bitfield
        interrupt::free(|_| {
            let rstr = unsafe {
                &(*RCC::ptr()).bdcr
            };
            rstr.modify(|_, w| w.bdrst().set_bit());
            rstr.modify(|_, w| w.bdrst().clear_bit());
        });
        self
    }
}

/// RTC kernel clock source selection
#[cfg(feature = "rtc")]
pub type RtcClkSel = crate::stm32::rcc::bdcr::RTCSEL_A;

#[cfg(feature = "rtc")]
impl Rtc {
    /// Returns true if the RTC is enabled.
    pub fn is_enabled(&self) -> bool {
        // unsafe: Owned exclusive access to this bitfield
        interrupt::free(|_| {
            let enr = unsafe {
                &(*RCC::ptr()).bdcr
            };
            enr.read().rtcen().bit_is_set()
        })
    }

    #[inline(always)]
    /// Modify a kernel clock for this
    /// peripheral. See RM0433 Section 8.5.8.
    ///
    /// **NOTE**: This can only be written one time per peripheral reset.
    /// Check `get_kernel_clk_mux()` to see if the write succeeded.
    pub fn kernel_clk_mux(self, sel: RtcClkSel) -> Self {
        // unsafe: Owned exclusive access to this bitfield
        interrupt::free(|_| {
            let ccip = unsafe {
                &(*RCC::ptr()).bdcr
            };
            ccip.modify(|_, w| w.rtcsel().variant(sel));
        });
        self
    }

    /// Return the current kernel clock selection
    pub fn get_kernel_clk_mux(&self) -> RtcClkSel {
        // unsafe: We only read from this bitfield
        let ccip = unsafe {
            &(*RCC::ptr()).bdcr
        };
        ccip.read().rtcsel().variant()
    }
}
