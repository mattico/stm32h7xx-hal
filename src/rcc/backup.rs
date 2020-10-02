//! Backup Power Domain

/// Backup Power Domain
#[allow(non_snake_case)]
pub struct Backup {
    #[cfg(feature = "rtc")]
    pub RTC: Rtc,
}

impl Backup {
    /// Creates a new `Backup`
    ///
    /// # Safety
    ///
    /// Must only be called once, after the backup domain write
    /// protection is disabled.
    pub(crate) unsafe fn new_singleton() -> Self {
        Self {
            #[cfg(feature = "rtc")]
            RTC: Rtc {
                _marker: core::marker::PhantomData,
            },
        }
    }
}

#[cfg(feature = "rtc")]
pub use rtc::{Rtc, RtcClkSel};

#[cfg(feature = "rtc")]
mod rtc {
    use crate::rcc::rec::ResetEnable;
    use crate::stm32::RCC;
    use core::marker::PhantomData;
    use cortex_m::interrupt;

    /// Reset, Enable and Clock functionality for RTC
    pub struct Rtc {
        pub(super) _marker: PhantomData<*const ()>,
    }

    unsafe impl Send for Rtc {}

    impl ResetEnable for Rtc {
        #[inline(always)]
        fn enable(self) -> Self {
            // unsafe: Owned exclusive access to this bitfield
            interrupt::free(|_| {
                let bdcr = unsafe { &(*RCC::ptr()).bdcr };
                bdcr.modify(|_, w| w.rtcen().set_bit());
            });
            self
        }
        #[inline(always)]
        fn disable(self) -> Self {
            // unsafe: Owned exclusive access to this bitfield
            interrupt::free(|_| {
                let bdcr = unsafe { &(*RCC::ptr()).bdcr };
                bdcr.modify(|_, w| w.rtcen().clear_bit());
            });
            self
        }
        #[inline(always)]
        fn reset(self) -> Self {
            // unsafe: Owned exclusive access to this bitfield
            interrupt::free(|_| {
                let bdcr = unsafe { &(*RCC::ptr()).bdcr };
                bdcr.modify(|_, w| w.bdrst().set_bit());
                bdcr.modify(|_, w| w.bdrst().clear_bit());
            });
            self
        }
    }

    /// RTC kernel clock source selection
    pub type RtcClkSel = crate::stm32::rcc::bdcr::RTCSEL_A;

    impl Rtc {
        /// Returns true if the RTC is enabled.
        pub fn is_enabled(&self) -> bool {
            // unsafe: Owned exclusive access to this bitfield
            interrupt::free(|_| {
                let bdcr = unsafe { &(*RCC::ptr()).bdcr };
                bdcr.read().rtcen().bit_is_set()
            })
        }

        #[inline(always)]
        /// Modify a kernel clock for this
        /// peripheral. See RM0433 Section 8.5.8.
        ///
        /// # NOTE
        ///
        /// This may only be written one time per peripheral reset.
        /// Check `get_kernel_clk_mux()` to see if the write succeeded.
        pub fn kernel_clk_mux(&mut self, sel: RtcClkSel) {
            // unsafe: Owned exclusive access to this bitfield
            interrupt::free(|_| {
                let bdcr = unsafe { &(*RCC::ptr()).bdcr };
                bdcr.modify(|_, w| w.rtcsel().variant(sel));
            });
        }

        /// Return the current kernel clock selection
        pub fn get_kernel_clk_mux(&self) -> RtcClkSel {
            // unsafe: We only read from this bitfield
            let bdcr = unsafe { &(*RCC::ptr()).bdcr };
            bdcr.read().rtcsel().variant()
        }
    }
}
