//! Example of configuring the real-time clock using the internal LSI oscillator

#![no_main]
#![no_std]

use log::info;

use cortex_m::asm;
use cortex_m_rt::entry;
use time::{date, time, PrimitiveDateTime};

use pac::interrupt;
use stm32h7xx_hal::{pac, prelude::*, rtc};

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
    let mut pwrcfg = pwr.freeze();

    // Take the backup power domain
    let backup = pwrcfg.backup().unwrap();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc.sys_ck(100.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - RTC");
    info!("");

    let mut rtc = rtc::Rtc::open_or_init(
        dp.RTC,
        backup.RTC,
        rtc::RtcClock::Lsi,
        rtc::TimeFormat::F24,
        &ccdr.clocks,
    );
    rtc.set_date_time(PrimitiveDateTime::new(
        date!(2020 - 01 - 01),
        time!(00:00),
    ));
    rtc.listen(rtc::Event::Wakeup);
    rtc.enable_wakeup(10);

    loop {
        info!("Time: {}", rtc.time().unwrap());
        rtc.unpend(rtc::Event::Wakeup);
        asm::wfi();
    }
}

#[interrupt]
fn RTC_WKUP() {}
