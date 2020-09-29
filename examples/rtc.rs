//! Example of configuring the real-time clock using an external 32768Hz Crystal Oscillator

#![no_main]
#![no_std]

use log::info;

use cortex_m_rt::entry;
use cortex_m::asm;
use time::{time, date, PrimitiveDateTime};

use stm32h7xx_hal::{pac, prelude::*, rtc};
use pac::interrupt;

#[path = "utilities/logger.rs"]
mod logger;

#[entry]
fn main() -> ! {
    logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Take the backup power domain
    let backup = pwrcfg.backup;

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc.sys_ck(100.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - RTC");
    info!("");

    let mut rtc = rtc::RtcBuilder::new(dp.RTC, backup.RTC, rtc::RtcClock::Lsi, rtc::TimeFormat::F24)
        .open(&ccdr.clocks)
        .or_else(|(rtc, _err)| rtc.init(&ccdr.clocks))
        .expect("Unable to initialize RTC");

    rtc.set_date_time(PrimitiveDateTime::new(date!(2020-01-01), time!(00:00)));
    rtc.listen(rtc::Event::Wakeup);
    rtc.enable_wakeup(10);

    loop {
        info!("Time: {}", rtc.time().unwrap());
        rtc.unpend(rtc::Event::Wakeup);
        asm::wfi();
    }
}

#[interrupt]
fn RTC_WKUP() {
}

