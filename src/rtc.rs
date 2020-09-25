// /*!
//   Real time clock

//   A continuously running clock that counts seconds. It is part of the backup domain which means
//   that the counter is not affected by system resets or standby mode. If Vbat is connected, it is
//   not reset even if the rest of the device is powered off. This allows it to be used to wake the
//   CPU when it is in low power mode.

//   Since it is part of the backup domain, write access to it must be enabled before the RTC can be
//   used. See `backup_domain` for more details.

//   See examples/rtc.rs and examples/blinky_rtc.rs for usage examples.
// */
// use crate::pac::{RCC, RTC};

// use crate::time::Hertz;

// use core::convert::Infallible;

// /**
//   Interface to the real time clock
// */
// pub struct Rtc {
//     reg: RTC,
// }

// impl Rtc {
//     /**
//       Initialises the RTC. The `BackupDomain` struct is created by
//       `Rcc.bkp.constrain()`.
//     */
//     pub fn rtc(reg: RTC, bkp: &mut BackupDomain) -> Self {
//         let mut result = Rtc { reg };

//         Rtc::enable_rtc(bkp);

//         // Set the prescaler to make it count up once every second.
//         let prl = LSE_HERTZ - 1;
//         assert!(prl < 1 << 20);
//         result.perform_write(|s| {
//             s.reg.prlh.write(|w| unsafe { w.bits(prl >> 16) });
//             s.reg.prll.write(|w| unsafe { w.bits(prl as u16 as u32) });
//         });

//         result
//     }

//     /// Enables the RTC device with the lse as the clock
//     fn enable_rtc(_bkp: &mut BackupDomain) {
//         // NOTE: Safe RCC access because we are only accessing bdcr
//         // and we have a &mut on BackupDomain
//         let rcc = unsafe { &*RCC::ptr() };
//         rcc.bdcr.modify(|_, w| {
//             w
//                 // start the LSE oscillator
//                 .lseon()
//                 .set_bit()
//                 // Enable the RTC
//                 .rtcen()
//                 .set_bit()
//                 // Set the source of the RTC to LSE
//                 .rtcsel()
//                 .lse()
//         })
//     }

//     /// Selects the frequency of the RTC Timer
//     /// NOTE: Maximum frequency of 16384 Hz using the internal LSE
//     pub fn select_frequency(&mut self, timeout: impl Into<Hertz>) {
//         let frequency = timeout.into().0;

//         // The manual says that the zero value for the prescaler is not recommended, thus the
//         // minimum division factor is 2 (prescaler + 1)
//         assert!(frequency <= LSE_HERTZ / 2);

//         let prescaler = LSE_HERTZ / frequency - 1;
//         self.perform_write(|s| {
//             s.reg.prlh.write(|w| unsafe { w.bits(prescaler >> 16) });
//             s.reg
//                 .prll
//                 .write(|w| unsafe { w.bits(prescaler as u16 as u32) });
//         });
//     }

//     /// Set the current RTC counter value to the specified amount
//     pub fn set_time(&mut self, counter_value: u32) {
//         self.perform_write(|s| {
//             s.reg
//                 .cnth
//                 .write(|w| unsafe { w.bits(counter_value >> 16) });
//             s.reg
//                 .cntl
//                 .write(|w| unsafe { w.bits(counter_value as u16 as u32) });
//         });
//     }

//     /**
//       Sets the time at which an alarm will be triggered

//       This also clears the alarm flag if it is set
//     */
//     pub fn set_alarm(&mut self, counter_value: u32) {
//         // Set alarm time
//         // See section 18.3.5 for explanation
//         let alarm_value = counter_value - 1;

//         // TODO: Remove this `allow` once these fields are made safe for stm32f100
//         #[allow(unused_unsafe)]
//         self.perform_write(|s| {
//             s.reg
//                 .alrh
//                 .write(|w| unsafe { w.alrh().bits((alarm_value >> 16) as u16) });
//             s.reg
//                 .alrl
//                 .write(|w| unsafe { w.alrl().bits(alarm_value as u16) });
//         });

//         self.clear_alarm_flag();
//     }

//     /// Enables the RTCALARM interrupt
//     pub fn listen_alarm(&mut self) {
//         // Enable alarm interrupt
//         self.perform_write(|s| {
//             s.reg.crh.modify(|_, w| w.alrie().set_bit());
//         })
//     }

//     /// Disables the RTCALARM interrupt
//     pub fn unlisten_alarm(&mut self) {
//         // Disable alarm interrupt
//         self.perform_write(|s| {
//             s.reg.crh.modify(|_, w| w.alrie().clear_bit());
//         })
//     }

//     /// Reads the current counter
//     pub fn current_time(&self) -> u32 {
//         // Wait for the APB1 interface to be ready
//         while !self.reg.crl.read().rsf().bit() {}

//         self.reg.cnth.read().bits() << 16 | self.reg.cntl.read().bits()
//     }

//     /// Enables the RTC second interrupt
//     pub fn listen_seconds(&mut self) {
//         self.perform_write(|s| s.reg.crh.modify(|_, w| w.secie().set_bit()))
//     }

//     /// Disables the RTC second interrupt
//     pub fn unlisten_seconds(&mut self) {
//         self.perform_write(|s| s.reg.crh.modify(|_, w| w.secie().clear_bit()))
//     }

//     /// Clears the RTC second interrupt flag
//     pub fn clear_second_flag(&mut self) {
//         self.perform_write(|s| s.reg.crl.modify(|_, w| w.secf().clear_bit()))
//     }

//     /// Clears the RTC alarm interrupt flag
//     pub fn clear_alarm_flag(&mut self) {
//         self.perform_write(|s| s.reg.crl.modify(|_, w| w.alrf().clear_bit()))
//     }

//     /**
//       Return `Ok(())` if the alarm flag is set, `Err(nb::WouldBlock)` otherwise.

//       ```rust
//       use nb::block;

//       rtc.set_alarm(rtc.read_counts() + 5);
//       // NOTE: Safe unwrap because Infallible can't be returned
//       block!(rtc.wait_alarm()).unwrap();
//       ```
//     */
//     pub fn wait_alarm(&mut self) -> nb::Result<(), Infallible> {
//         if self.reg.crl.read().alrf().bit() {
//             self.reg.crl.modify(|_, w| w.alrf().clear_bit());
//             Ok(())
//         } else {
//             Err(nb::Error::WouldBlock)
//         }
//     }

//     /**
//       The RTC registers can not be written to at any time as documented on page
//       485 of the manual. Performing writes using this function ensures that
//       the writes are done correctly.
//     */
//     fn perform_write(&mut self, func: impl Fn(&mut Self)) {
//         // Wait for the last write operation to be done
//         while !self.reg.crl.read().rtoff().bit() {}
//         // Put the clock into config mode
//         self.reg.crl.modify(|_, w| w.cnf().set_bit());

//         // Perform the write operation
//         func(self);

//         // Take the device out of config mode
//         self.reg.crl.modify(|_, w| w.cnf().clear_bit());
//         // Wait for the write to be done
//         while !self.reg.crl.read().rtoff().bit() {}
//     }
// }

use crate::rcc::rec::{self, ResetEnable};
use crate::rcc::CoreClocks;
use crate::stm32::RTC;
use crate::time::Hertz;

pub enum Event {
    AlarmA,
    AlarmB,
    Wakeup,
    Timestamp,
    Tamper1,
    Tamper2,
    Tamper3,
}

pub trait RtcExt {
    fn constrain(self) -> Rtc;
}

impl RtcExt for RTC {
    fn constrain(self) -> Rtc {
        Rtc { reg: self }
    }
}

pub struct Rtc {
    reg: RTC,
}

impl Rtc {
    pub fn rtc(rtc: RTC, prec: rec::Rtc, clocks: &CoreClocks) -> Self {
        // TODO: should we reset here?
        let prec = prec.enable().reset();

        let ker_ck = match prec.get_kernel_clk_mux() {
            rec::RtcClkSel::NOCLOCK => None,
            rec::RtcClkSel::LSI => clocks.lsi_ck(),
            rec::RtcClkSel::LSE => clocks.lse_ck(),
            rec::RtcClkSel::HSE => clocks.hse_ck().map(|x| Hertz(x.0 / 8)),
        }
        .expect("rtc_ker_ck not running!");

        // Disable register write protection
        rtc.wpr.write(|w| w.bits(0xCA));
        rtc.wpr.write(|w| w.bits(0x53));

        Self { reg: rtc }
    }

    pub fn read_backup_reg(&self, reg: u8) -> u32 {
        //self.reg.bkp[reg].read().bits()
        todo!()
    }

    pub fn write_backup_reg(&mut self, reg: u8, value: u32) {
        //self.reg.bkp[reg].write(|w| w.bits(value));
    }

    pub fn date(&self) -> Date {
        Date { data: self.reg.dr.read().bits() }
    }

    pub fn time(&self) -> Time {
        Time { data: self.reg.tr.read().bits() }
    }

    pub fn subseconds(&self) -> f32 {
        let ss = self.reg.ssr.read().bits() as f32;
        let prediv_s = self.reg.prer.read().prediv_s().bits() as f32;
        (prediv_s - ss) / (prediv_s + 1.0) 
    }

    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Tamper1 | Event::Tamper2 | Event::Tamper3 => {
                self.reg.tampcr.modify(|_, w| match event {
                    Event::Tamper1 => w.tamp1ie().set_bit(),
                    Event::Tamper2 => w.tamp2ie().set_bit(),
                    Event::Tamper3 => w.tamp3ie().set_bit(),
                    _ => unreachable!(),
                })
            }
            _ => self.reg.cr.modify(|_, w| match event {
                Event::AlarmA => w.alraie().set_bit(),
                Event::AlarmB => w.alrbie().set_bit(),
                Event::Wakeup => w.wutie().set_bit(),
                Event::Timestamp => w.tsie().set_bit(),
                _ => unreachable!(),
            }),
        }
    }

    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Tamper1 | Event::Tamper2 | Event::Tamper3 => {
                self.reg.tampcr.modify(|_, w| match event {
                    Event::Tamper1 => w.tamp1ie().clear_bit(),
                    Event::Tamper2 => w.tamp2ie().clear_bit(),
                    Event::Tamper3 => w.tamp3ie().clear_bit(),
                    _ => unreachable!(),
                })
            }
            _ => self.reg.cr.modify(|_, w| match event {
                Event::AlarmA => w.alraie().clear_bit(),
                Event::AlarmB => w.alrbie().clear_bit(),
                Event::Wakeup => w.wutie().clear_bit(),
                Event::Timestamp => w.tsie().clear_bit(),
                _ => unreachable!(),
            }),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub enum DayOfWeek {
    Monday = 1,
    Tuesday = 2,
    Wednesday = 3,
    Thursday = 4,
    Friday = 5,
    Saturday = 6,
    Sunday = 7,
}

#[derive(Copy, Clone, PartialEq)]
pub struct Date {
    data: u32,
}

impl Date {
    pub fn year(&self) -> u16 {
        let tens = (self.data >> 20 & 0b1111) as u16;
        let ones = (self.data >> 16 & 0b1111) as u16;
        2000 + tens * 10 + ones
    }

    pub fn month(&self) -> u8 {
        let tens = (self.data >> 12 & 0b1 == 1) as u8;
        let ones = (self.data >> 8 & 0b1111) as u8;
        tens * 10 + ones
    }

    pub fn day_of_week(&self) -> DayOfWeek {
        let ones = self.data >> 13 & 0b111;
        match ones {
            1 => DayOfWeek::Monday,
            2 => DayOfWeek::Tuesday,
            3 => DayOfWeek::Wednesday,
            4 => DayOfWeek::Thursday,
            5 => DayOfWeek::Friday,
            6 => DayOfWeek::Saturday,
            7 => DayOfWeek::Sunday,
            _ => unreachable!(),
        }
    }

    pub fn day_of_month(&self) -> u8 {
        let tens = (self.data >> 4 & 0b11) as u8;
        let ones = (self.data & 0b1111) as u8;
        tens * 10 + ones
    }
}

pub struct Time {
    data: u32,
}

impl Time {
    /// Returns `true` if the `Time` is PM or `false` if it is AM or 24-hour time.
    pub fn is_pm(&self) -> bool {
        self.data & (1 << 22) != 0
    }

    pub fn hours(&self) -> u8 {
        let tens = (self.data >> 20 & 0b11) as u8;
        let ones = (self.data >> 16 & 0b1111) as u8;
        tens * 10 + ones
    }

    pub fn minutes(&self) -> u8 {
        let tens = (self.data >> 12 & 0b111) as u8;
        let ones = (self.data >> 8 & 0b1111) as u8;
        tens * 10 + ones
    }

    pub fn seconds(&self) -> u8 {
        let tens = (self.data >> 4 & 0b111) as u8;
        let ones = (self.data & 0b1111) as u8;
        tens * 10 + ones
    }
}
