//! Real-Time Clock

use core::convert::TryInto;

use time;

use crate::backup;
use crate::rcc::rec::ResetEnable;
use crate::rcc::CoreClocks;
use crate::stm32::{RCC, RTC};
use crate::time::Hertz;

pub enum Event {
    AlarmA,
    AlarmB,
    Wakeup,
    Timestamp,
    LseCss,
}

#[derive(Copy, Clone, PartialEq)]
pub enum RtcClock {
    /// LSE (Low-Speed External)
    Lse {
        freq: Hertz,
        bypass: bool,
        css: bool,
    },
    /// LSI (Low-Speed Internal)
    Lsi,
    /// HSE (High-Speed External) Divided by 32
    HseDiv32,
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq)]
pub enum TimeFormat {
    F24 = 0,
    AmPm = 1,
}

pub struct Config {
    pub clock_source: RtcClock,
    pub format: TimeFormat,
}

pub struct RtcBuilder {
    rtc: RTC,
    prec: backup::Rtc,
    clock_source: RtcClock,
    format: TimeFormat,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Error {
    RtcNotRunning,
    ClockNotRunning,
    ConfigMismatch,
    ClockTooFast,
    CalendarNotInitialized,
}

impl RtcBuilder {
    pub fn new(rtc: RTC, prec: backup::Rtc, clock_source: RtcClock, format: TimeFormat) -> Self {
        Self {
            rtc,
            prec,
            clock_source,
            format,
        }
    }

    /// Opens the RTC if it is running and its configuration matches the `RtcBuilder`.
    pub fn open(self, clocks: &CoreClocks) -> Result<Rtc, (Self, Error)> {
        if !self.prec.is_enabled() {
            return Err((self, Error::RtcNotRunning));
        }

        let bdcr = unsafe { (&*RCC::ptr()).bdcr.read() };

        let clock_source_matches = match (self.clock_source, self.prec.get_kernel_clk_mux()) {
            (RtcClock::Lsi, backup::RtcClkSel::LSI) => true,
            (RtcClock::HseDiv32, backup::RtcClkSel::HSE) => true,
            (RtcClock::Lse { bypass, css, .. }, backup::RtcClkSel::LSE) => {
                bypass == bdcr.lsebyp().is_bypassed() && css == bdcr.lsecsson().is_security_on()
            }
            _ => false,
        };
        if !clock_source_matches {
            return Err((self, Error::ConfigMismatch));
        }

        let clock_source_running = match self.clock_source {
            RtcClock::Lsi => clocks.lsi_ck().is_some(),
            RtcClock::HseDiv32 => clocks.hse_ck().is_some(),
            RtcClock::Lse { .. } => bdcr.lserdy().is_ready(),
        };
        if !clock_source_running {
            return Err((self, Error::ClockNotRunning));
        }

        if self.rtc.isr.read().inits().bit_is_clear() {
            return Err((self, Error::CalendarNotInitialized));
        }

        if self.rtc.cr.read().fmt().bit() as u8 != self.format as u8 {
            return Err((self, Error::ConfigMismatch));
        }

        Ok(Rtc {
            reg: self.rtc,
            format: self.format,
        })
    }

    /// Resets the RTC, including the backup registers, then initializes it.
    pub fn init(self, clocks: &CoreClocks) -> Result<Rtc, Error> {
        let RtcBuilder {
            rtc,
            prec,
            clock_source,
            format,
        } = self;

        let prec = prec.reset().enable();

        let bdcr = unsafe { &(*RCC::ptr()).bdcr };

        // Initialize LSE if required
        if let RtcClock::Lse { bypass, .. } = clock_source {
            // Ensure LSE is on and stable
            bdcr.modify(|_, w| w.lseon().on().lsebyp().bit(bypass));
            while bdcr.read().lserdy().is_not_ready() {}
        }

        // Select RTC kernel clock
        prec.kernel_clk_mux(match clock_source {
            RtcClock::HseDiv32 => backup::RtcClkSel::HSE,
            RtcClock::Lsi => backup::RtcClkSel::LSI,
            RtcClock::Lse { .. } => backup::RtcClkSel::LSE,
        });

        // Now we can enable CSS, if required
        if let RtcClock::Lse { css: true, .. } = clock_source {
            bdcr.modify(|_, w| w.lsecsson().security_on());
        }

        let ker_ck = match clock_source {
            RtcClock::Lse { freq, .. } => Some(freq),
            RtcClock::Lsi => clocks.lsi_ck(),
            RtcClock::HseDiv32 => clocks.hse_ck().map(|x| Hertz(x.0 / 32)),
        }
        .ok_or(Error::ClockNotRunning)?
        .0;
        if ker_ck > (1 << 22) {
            return Err(Error::ClockTooFast);
        }

        // Disable RTC register write protection
        rtc.wpr.write(|w| unsafe { w.bits(0xCA) });
        rtc.wpr.write(|w| unsafe { w.bits(0x53) });

        // Enter initialization mode
        rtc.isr.modify(|_, w| w.init().set_bit());
        while rtc.isr.read().initf().bit_is_clear() {}

        // Configure prescaler for 1Hz clock
        // Want to maximize a_pre_max for power reasons, though it reduces the
        // subsecond precision.
        let total_div = ker_ck;
        let a_pre_max = 1 << 7;
        let s_pre_max = 1 << 15;

        let (a_pre, s_pre) = if total_div <= a_pre_max {
            (total_div, 0)
        } else if total_div % a_pre_max == 0 {
            (a_pre_max, total_div / a_pre_max)
        } else {
            todo!()
        };

        rtc.prer
            .write(|w| unsafe { w.prediv_s().bits(s_pre as u16).prediv_a().bits(a_pre as u8) });

        // Set time format
        rtc.cr
            .modify(|_, w| w.fmt().bit(format == TimeFormat::AmPm));

        // Exit initialization mode
        rtc.isr.modify(|_, w| w.init().clear_bit());

        Ok(Rtc { reg: rtc, format })
    }
}

pub struct Rtc {
    reg: RTC,
    format: TimeFormat,
}

impl Rtc {
    pub fn read_backup_reg(&self, reg: u8) -> u32 {
        match reg {
            0 => self.reg.bkp0r.read().bkp().bits(),
            1 => self.reg.bkp1r.read().bkp().bits(),
            2 => self.reg.bkp2r.read().bkp().bits(),
            3 => self.reg.bkp3r.read().bkp().bits(),
            4 => self.reg.bkp4r.read().bkp().bits(),
            5 => self.reg.bkp5r.read().bkp().bits(),
            6 => self.reg.bkp6r.read().bkp().bits(),
            7 => self.reg.bkp7r.read().bkp().bits(),
            8 => self.reg.bkp8r.read().bkp().bits(),
            9 => self.reg.bkp9r.read().bkp().bits(),
            10 => self.reg.bkp10r.read().bkp().bits(),
            11 => self.reg.bkp11r.read().bkp().bits(),
            12 => self.reg.bkp12r.read().bkp().bits(),
            13 => self.reg.bkp13r.read().bkp().bits(),
            14 => self.reg.bkp14r.read().bkp().bits(),
            15 => self.reg.bkp15r.read().bkp().bits(),
            16 => self.reg.bkp16r.read().bkp().bits(),
            17 => self.reg.bkp17r.read().bkp().bits(),
            18 => self.reg.bkp18r.read().bkp().bits(),
            19 => self.reg.bkp19r.read().bkp().bits(),
            20 => self.reg.bkp20r.read().bkp().bits(),
            21 => self.reg.bkp21r.read().bkp().bits(),
            22 => self.reg.bkp22r.read().bkp().bits(),
            23 => self.reg.bkp23r.read().bkp().bits(),
            24 => self.reg.bkp24r.read().bkp().bits(),
            25 => self.reg.bkp25r.read().bkp().bits(),
            26 => self.reg.bkp26r.read().bkp().bits(),
            27 => self.reg.bkp27r.read().bkp().bits(),
            28 => self.reg.bkp28r.read().bkp().bits(),
            29 => self.reg.bkp29r.read().bkp().bits(),
            30 => self.reg.bkp30r.read().bkp().bits(),
            31 => self.reg.bkp31r.read().bkp().bits(),
            _ => panic!("Backup reg index out of range"),
        }

        // TODO: stm32h7 0.13.0
        //self.reg.bkp[reg].read().bits()
    }

    pub fn write_backup_reg(&mut self, reg: u8, value: u32) {
        match reg {
            0 => self.reg.bkp0r.write(|w| unsafe { w.bkp().bits(value) }),
            1 => self.reg.bkp1r.write(|w| unsafe { w.bkp().bits(value) }),
            2 => self.reg.bkp2r.write(|w| unsafe { w.bkp().bits(value) }),
            3 => self.reg.bkp3r.write(|w| unsafe { w.bkp().bits(value) }),
            4 => self.reg.bkp4r.write(|w| unsafe { w.bkp().bits(value) }),
            5 => self.reg.bkp5r.write(|w| unsafe { w.bkp().bits(value) }),
            6 => self.reg.bkp6r.write(|w| unsafe { w.bkp().bits(value) }),
            7 => self.reg.bkp7r.write(|w| unsafe { w.bkp().bits(value) }),
            8 => self.reg.bkp8r.write(|w| unsafe { w.bkp().bits(value) }),
            9 => self.reg.bkp9r.write(|w| unsafe { w.bkp().bits(value) }),
            10 => self.reg.bkp10r.write(|w| unsafe { w.bkp().bits(value) }),
            11 => self.reg.bkp11r.write(|w| unsafe { w.bkp().bits(value) }),
            12 => self.reg.bkp12r.write(|w| unsafe { w.bkp().bits(value) }),
            13 => self.reg.bkp13r.write(|w| unsafe { w.bkp().bits(value) }),
            14 => self.reg.bkp14r.write(|w| unsafe { w.bkp().bits(value) }),
            15 => self.reg.bkp15r.write(|w| unsafe { w.bkp().bits(value) }),
            16 => self.reg.bkp16r.write(|w| unsafe { w.bkp().bits(value) }),
            17 => self.reg.bkp17r.write(|w| unsafe { w.bkp().bits(value) }),
            18 => self.reg.bkp18r.write(|w| unsafe { w.bkp().bits(value) }),
            19 => self.reg.bkp19r.write(|w| unsafe { w.bkp().bits(value) }),
            20 => self.reg.bkp20r.write(|w| unsafe { w.bkp().bits(value) }),
            21 => self.reg.bkp21r.write(|w| unsafe { w.bkp().bits(value) }),
            22 => self.reg.bkp22r.write(|w| unsafe { w.bkp().bits(value) }),
            23 => self.reg.bkp23r.write(|w| unsafe { w.bkp().bits(value) }),
            24 => self.reg.bkp24r.write(|w| unsafe { w.bkp().bits(value) }),
            25 => self.reg.bkp25r.write(|w| unsafe { w.bkp().bits(value) }),
            26 => self.reg.bkp26r.write(|w| unsafe { w.bkp().bits(value) }),
            27 => self.reg.bkp27r.write(|w| unsafe { w.bkp().bits(value) }),
            28 => self.reg.bkp28r.write(|w| unsafe { w.bkp().bits(value) }),
            29 => self.reg.bkp29r.write(|w| unsafe { w.bkp().bits(value) }),
            30 => self.reg.bkp30r.write(|w| unsafe { w.bkp().bits(value) }),
            31 => self.reg.bkp31r.write(|w| unsafe { w.bkp().bits(value) }),
            _ => panic!("Backup reg index out of range"),
        }

        // TODO: stm32h7 0.13.0
        //self.reg.bkp[reg].write(|w| w.bits(value));
    }

    pub fn set_date_time(&mut self, date: time::PrimitiveDateTime) {
        // Enter initialization mode
        self.reg.isr.modify(|_, w| w.init().set_bit());
        while self.reg.isr.read().initf().bit_is_clear() {}

        let mut hour = date.hour();
        let pm = self.format == TimeFormat::AmPm && hour >= 12;
        if pm {
            hour -= 12;
        }
        let ht = hour / 10;
        let hu = hour % 10;

        let minute = date.minute();
        let mnt = minute / 10;
        let mnu = minute % 10;

        let second = date.second();
        let st = second / 10;
        let su = second % 10;

        self.reg.tr.write(|w| unsafe {
            w.pm()
                .bit(pm)
                .ht()
                .bits(ht)
                .hu()
                .bits(hu)
                .mnt()
                .bits(mnt)
                .mnu()
                .bits(mnu)
                .st()
                .bits(st)
                .su()
                .bits(su)
        });

        let year = date.year();
        let yt = ((year - 2000) / 10) as u8;
        let yu = ((year - 2000) % 10) as u8;

        let wdu = date.weekday().number_from_monday();

        let (month, day) = date.month_day();

        let mt = month > 9;
        let mu = month % 10;

        let dt = day / 10;
        let du = day % 10;

        self.reg.dr.write(|w| unsafe {
            w.yt()
                .bits(yt)
                .yu()
                .bits(yu)
                .wdu()
                .bits(wdu)
                .mt()
                .bit(mt)
                .mu()
                .bits(mu)
                .dt()
                .bits(dt)
                .du()
                .bits(du)
        });

        // Exit initialization mode
        self.reg.isr.modify(|_, w| w.init().clear_bit());
    }

    /// Wait for initialization or shift to complete
    fn wait_for_sync(&self) {
        while self.reg.isr.read().rsf().bit_is_clear() {}
    }

    pub fn date(&self) -> Option<time::Date> {
        self.wait_for_sync();
        let data = self.reg.dr.read();
        let year = 2000 + data.yt().bits() as i32 * 10 + data.yu().bits() as i32;
        let month = data.mt().bits() as u8 * 10 + data.mu().bits();
        let day = data.dt().bits() as u8 * 10 + data.du().bits();
        let date =
            time::Date::try_from_ymd(year, month, day).expect("Invalid data in RTC date register");
        Some(date)
    }

    pub fn time(&self) -> Option<time::Time> {
        self.wait_for_sync();
        let data = self.reg.tr.read();
        let mut hour = data.ht().bits() * 10 + data.hu().bits();
        if data.pm().bit_is_set() {
            hour += 12;
        }
        let minute = data.mnt().bits() * 10 + data.mnu().bits();
        let second = data.st().bits() * 10 + data.su().bits();
        let micro = self.subsec_micros();
        let time = time::Time::try_from_hms_micro(hour, minute, second, micro)
            .expect("Invalid data in RTC time register");
        Some(time)
    }

    pub fn date_time(&self) -> Option<time::PrimitiveDateTime> {
        let date = self.date()?;
        let time = self.time()?;
        Some(time::PrimitiveDateTime::new(date, time))
    }

    /// Returns the fraction of seconds that have occurred since the last second tick.
    /// The precision of this value depends on the value of the synchronous prescale divider
    /// (prediv_s) which depends on the frequency of the RTC clock. E.g. with a 32,768 Hz
    /// crystal this value has a resolution of 1/256 of a second.
    pub fn subseconds(&self) -> f32 {
        self.wait_for_sync();
        let ss = self.reg.ssr.read().bits() as f32;
        let prediv_s = self.reg.prer.read().prediv_s().bits() as f32;
        (prediv_s - ss) / (prediv_s + 1.0)
    }

    /// Returns the fraction of seconds that have occurred since the last second tick
    /// as a number of milliseconds rounded to the nearest whole number.
    pub fn subsec_micros(&self) -> u32 {
        self.wait_for_sync();
        let ss = self.reg.ssr.read().ss().bits() as u32;
        let prediv_s = self.reg.prer.read().prediv_s().bits() as u32;
        ((prediv_s - ss) * 1_000) / (prediv_s + 1)
    }

    /// Returns the raw value of the synchronous subsecond counter
    ///
    /// This counts up to `self.subsec_res()` then resets to zero once per second.
    pub fn subsec_raw(&self) -> u16 {
        self.wait_for_sync();
        self.reg.ssr.read().ss().bits()
    }

    /// Returns the resolution of subsecond values
    ///
    /// The RTC counter increments this number of times per second.
    pub fn subsec_res(&self) -> u16 {
        self.reg.prer.read().prediv_s().bits()
    }

    /// Configures the wakeup timer to trigger periodically after `interval` seconds
    ///
    /// **NOTE:** Panics if interval is greater than 2^17-1
    pub fn enable_wakeup(&mut self, interval: u32) {
        self.reg.cr.modify(|_, w| w.wute().clear_bit());
        self.reg.isr.modify(|_, w| w.wutf().clear_bit());
        while self.reg.isr.read().wutwf().bit_is_clear() {}

        if interval > 1 << 16 {
            self.reg
                .cr
                .modify(|_, w| unsafe { w.wucksel().bits(0b110) });
            let interval: u16 = (interval - 1 << 16)
                .try_into()
                .expect("Interval was too large for wakeup timer");
            self.reg.wutr.write(|w| unsafe { w.wut().bits(interval) });
        } else {
            self.reg
                .cr
                .modify(|_, w| unsafe { w.wucksel().bits(0b110) });
            let interval: u16 = interval
                .try_into()
                .expect("Interval was too large for wakeup timer");
            self.reg.wutr.write(|w| unsafe { w.wut().bits(interval) });
        }

        self.reg.cr.modify(|_, w| w.wute().set_bit());
    }

    pub fn disable_wakeup(&mut self) {
        self.reg.cr.modify(|_, w| w.wute().clear_bit());
        self.reg.isr.modify(|_, w| w.wutf().clear_bit());
    }

    /// Configures the timestamp to be captured when the RTC switches to Vbat power
    pub fn enable_vbat_timestamp(&mut self) {
        self.reg.cr.modify(|_, w| w.tse().clear_bit());
        self.reg.isr.modify(|_, w| w.tsf().clear_bit());
        self.reg.cr.modify(|_, w| w.itse().set_bit());
        self.reg.cr.modify(|_, w| w.tse().set_bit());
    }

    /// Disables the timestamp
    pub fn disable_timestamp(&mut self) {
        self.reg.cr.modify(|_, w| w.tse().clear_bit());
        self.reg.isr.modify(|_, w| w.tsf().clear_bit());
    }

    // TODO: Timestamp overflow flag
    /// Reads the stored value of the timestamp if present
    ///
    /// Clears the timestamp interrupt flags.
    pub fn read_timestamp(&self) -> Option<time::PrimitiveDateTime> {
        if self.reg.isr.read().tsf().bit_is_set() {
            let data = self.reg.dr.read();
            let year = 2000 + data.yt().bits() as i32 * 10 + data.yu().bits() as i32;
            let month = data.mt().bits() as u8 * 10 + data.mu().bits();
            let day = data.dt().bits() * 10 + data.du().bits();
            let date = time::Date::try_from_ymd(year, month, day)
                .expect("Invalid data in RTC timestamp date register");
            let data = self.reg.tr.read();
            let mut hour = data.ht().bits() * 10 + data.hu().bits();
            if data.pm().bit_is_set() {
                hour += 12;
            }
            let minute = data.mnt().bits() * 10 + data.mnu().bits();
            let second = data.st().bits() * 10 + data.su().bits();
            let micro = self.subsec_micros();
            let time = time::Time::try_from_hms_micro(hour, minute, second, micro)
                .expect("Invalid data in RTC timestamp time register");

            self.reg
                .isr
                .modify(|_, w| w.tsf().clear_bit().itsf().clear_bit());

            Some(time::PrimitiveDateTime::new(date, time))
        } else {
            None
        }
    }

    // TODO: Alarms

    /// Start listening for `event`
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::LseCss => unsafe {
                (&*RCC::ptr()).cier.modify(|_, w| w.lsecssie().enabled());
            },
            Event::AlarmA => self.reg.cr.modify(|_, w| w.alraie().set_bit()),
            Event::AlarmB => self.reg.cr.modify(|_, w| w.alrbie().set_bit()),
            Event::Wakeup => self.reg.cr.modify(|_, w| w.wutie().set_bit()),
            Event::Timestamp => self.reg.cr.modify(|_, w| w.tsie().set_bit()),
        }
    }

    /// Stop listening for `event`
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::LseCss => unsafe {
                (&*RCC::ptr()).cier.modify(|_, w| w.lsecssie().disabled());
            },
            Event::AlarmA => self.reg.cr.modify(|_, w| w.alraie().clear_bit()),
            Event::AlarmB => self.reg.cr.modify(|_, w| w.alrbie().clear_bit()),
            Event::Wakeup => self.reg.cr.modify(|_, w| w.wutie().clear_bit()),
            Event::Timestamp => self.reg.cr.modify(|_, w| w.tsie().clear_bit()),
        }
    }

    /// Returns `true` if `event` is pending
    pub fn is_pending(&self, event: Event) -> bool {
        match event {
            Event::LseCss => unsafe { (&*RCC::ptr()).cifr.read().lsecssf().bit_is_set() },
            Event::AlarmA => self.reg.isr.read().alraf().bit_is_set(),
            Event::AlarmB => self.reg.isr.read().alrbf().bit_is_set(),
            Event::Wakeup => self.reg.isr.read().wutf().bit_is_set(),
            Event::Timestamp => self.reg.isr.read().tsf().bit_is_set(),
        }
    }

    /// Clears the interrupt flag for `event`
    pub fn unpend(&mut self, event: Event) {
        match event {
            Event::LseCss => unsafe { (&*RCC::ptr()).cicr.write(|w| w.lsecssc().clear()) },
            Event::AlarmA => self.reg.isr.modify(|_, w| w.alraf().clear_bit()),
            Event::AlarmB => self.reg.isr.modify(|_, w| w.alrbf().clear_bit()),
            Event::Wakeup => self.reg.isr.modify(|_, w| w.wutf().clear_bit()),
            Event::Timestamp => self.reg.isr.modify(|_, w| w.tsf().clear_bit()),
        }
    }
}
