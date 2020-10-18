//! Demonstrates using a DMA memory-to-peripheral transfer to transmit data over USART 3.
//! On an ST NUCLEO-H743ZI2 board USART3 is connected to the onboard ST-LINK's Virtual COM
//! Port by default.

#![no_main]
#![no_std]

#[path = "utilities/logger.rs"]
mod logger;

use bbqueue::{BBBuffer, ConstBBBuffer, Consumer, GrantR, Producer, consts::*};
use core::fmt::{self, Write};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, pre_init};
use log::info;
use stm32h7xx_hal::{pac, prelude::*};
use stm32h7xx_hal::interrupt;
use stm32h7xx_hal::rcc::ResetEnable;


// DMA2 can only read from memory in its domain (D2), so we can't use regular stack/heap.
// Use SRAM3 to store our queue instead.
#[link_section = ".sram3"]
static mut QUEUE: MaybeUninit<BBBuffer<U8192>> = BBBuffer(ConstBBBuffer::new());

static DMA: Mutex<RefCell<Option<DmaState>>> = Mutex::new(RefCell::new(None));
static PROD: Mutex<RefCell<Option<Producer<'static, U8192>>>> = Mutex::new(RefCell::new(None));

struct DmaState {
    dma: pac::DMA2,
    uart: pac::USART3,
    cons: Consumer<'static, U8192>,
    grant: Option<GrantR<'static, U8192>>,
}

impl DmaState {
    pub fn start_dma_tx(&mut self) {   
        if let Ok(grant) = self.cons.read() {
            // Configure memory source for DMA
            self.dma.st[1].m0ar.write(|w| w.m0a().bits(grant.as_ptr() as u32));
            self.dma.st[1].ndtr.write(|w| w.ndt().bits(grant.len() as u16));

            // Configure DMA destination
            let usart3_tdr = pac::USART3::ptr() as u32 + 0x28;
            self.dma.st[1].par.write(|w| w.pa().bits(usart3_tdr));

            // Configure DMA parameters
            self.dma.st[1].cr.write(|w| {
                unsafe { w.bits(1 << 20) } // TRBUFF enable, required for UARTs
                    .dir().memory_to_peripheral()
                    .minc().incremented()
                    .pinc().fixed()
                    .msize().bits8()
                    .psize().bits8()
                    .pl().low()
                    .teie().enabled()
                    .dmeie().enabled()
                    .tcie().enabled()
                    .htie().disabled()
                    .pfctrl().dma()
                    .circ().disabled()
            });

            // Disable DMA FIFO
            self.dma.st[1].fcr.write(|w| w.dmdis().enabled().feie().enabled());

            // Clear all interrupts for stream
            self.dma.lifcr.write(|w| {
                w.cdmeif1().clear()
                    .cfeif1().clear()
                    .chtif1().clear()
                    .ctcif1().clear()
                    .cteif1().clear()
            });
            
            self.dma.st[1].cr.modify(|_, w| w.en().enabled()); // Enable stream
            while self.dma.st[1].cr.read().en().is_disabled() {}

            self.uart.cr3.write(|w| w.dmat().enabled()); // Enable UART DMA
    
            // Store grant so interrupt can check on progress
            assert!(self.grant.is_none());
            self.grant.replace(grant);
        }
    }

    pub fn complete_dma(&mut self) {
        if let Some(grant) = self.grant.take() {
            grant.release(usize::MAX); // Release read memory back to queue
    
            self.uart.cr3.write(|w| w.dmat().disabled()); // Disable UART DMA
    
            // Disable interrupts
            self.dma.st[1].cr.modify(|_, w| {
                w.teie().disabled()
                    .dmeie().disabled()
                    .tcie().disabled()
                    .htie().disabled()
            });
    
            // Clear interrupt flags
            self.dma.lifcr.write(|w| {
                w.ctcif1().clear()
                    .chtif1().clear()
                    .cteif1().clear()
                    .cdmeif1().clear()
                    .cfeif1().clear()
            });
        } else {
            unreachable!();
        }
    }
}

/// Writes the bytes to the DMA buffer. Returns the number of bytes written, or an error if 
/// nothing could be written.
pub fn write(data: &[u8]) -> nb::Result<usize, bbqueue::Error> {
    if data.len() == 0 {
        return Ok(0);
    }

    let (buffer_full, len) = cortex_m::interrupt::free(|cs| {
        let mut rc = PROD.borrow(cs).borrow_mut();
        let prod = rc.as_mut().unwrap();
        if let Ok(mut grant) = prod.grant_exact(data.len()) {
            grant.copy_from_slice(data);
            grant.commit(usize::MAX);
            Ok((false, data.len()))
        } else {
            match prod.grant_max_remaining(data.len()) {
                Ok(mut grant) => {
                    let len = grant.len();
                    grant.copy_from_slice(&data[..len]);
                    grant.commit(usize::MAX);
                    Ok((true, len))
                }
                Err(bbqueue::Error::InsufficientSize) => Err(nb::Error::WouldBlock),
                Err(e) => Err(e.into()),
            }
        }
    })?;

    // Line buffering unless we somehow fill the buffer without starting DMA
    if buffer_full || data.contains(&b'\n') {
        cortex_m::interrupt::free(|cs| {
            let mut rc = DMA.borrow(cs).borrow_mut();
            let mut state = rc.take().unwrap();
            let dma_running = state.dma.st[1].cr.read().en().is_enabled();

            if !dma_running {
                state.start_dma_tx();
            }

            rc.replace(state);
        });
    }

    Ok(len)
}

/// Writes all of the data to the DMA buffer, blocking until it is complete.
pub fn write_all(mut buf: &[u8]) -> Result<(), bbqueue::Error> {
    while !buf.is_empty() {
        let n = nb::block!(write(buf))?;
        buf = &buf[n..];
    }
    Ok(())
}

pub struct Writer();

impl fmt::Write for Writer {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        write_all(s.as_bytes()).map_err(|_| fmt::Error)?;
        Ok(())
    }
}

#[pre_init]
unsafe fn pre_init() {
    // QUEUE is in .sram3 not .data, so it doesn't get initialized by the runtime.
    // For BBQueue 0.4.10, all zero is the correct initialization.
    // We do this in pre_init so the compiler doesn't eliminate it or reorder it,
    // and so it's harder to misuse the queue in main().
    // Alternatively use https://github.com/jamesmunns/bbqueue/pull/70
    let mut ptr = 0x30040000 as *mut u32;
    let end = ptr.offset(core::mem::size_of::<BBBuffer<U8192>>() as isize / 4);
    while ptr < end {
        core::ptr::write_volatile(ptr, 0);
        ptr = ptr.offset(1);
    }
}

#[entry]
fn main() -> ! {
    logger::init();
    let mut cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Enable CPU caches
    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Enable SRAM3
    info!("Setup SRAM3...                ");
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().enabled());

    info!("Setup MPU...                  ");
    // Disable cache for QUEUE so writes to it don't pollute dcache and actually complete before DMA reads.
    // Note the buffer in QUEUE may not be laid out at the beginning of the struct so you can't shrink
    // the region to just the size of the buffer.
    // https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
    unsafe {
        cp.MPU.ctrl.write(0b101); // Enable MPU and Default Memory Map
        cp.MPU.rnr.write(0); // Region 0
        cp.MPU.rbar.write(&QUEUE as *const _ as u32); // SRAM3 Address
        cp.MPU.rasr.write(
            1 << 28 // Disable instruction fetch
            | 0b011 << 24 // RW/RW
            | 0b001000 << 16 // Regular memory, non-cacheable
            | 13 << 1 // Size: 2^14 = 16K
            | 1, // Enabled
        );
    }

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz())
        .pclk1(80.mhz())
        .freeze(vos, &dp.SYSCFG);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    info!("Setup USART3...               ");

    // Configure UART
    let tx = gpiod.pd8.into_alternate_af7();
    let rx = gpiod.pd9.into_alternate_af7();
    let uart = dp.USART3.serial((tx, rx), 115_200.bps(), ccdr.peripheral.USART3, &ccdr.clocks).unwrap();
    let uart = uart.release(); // Get access to the raw peripheral after configuring

    info!("Enable DMA...                  ");

    // Enable DMA2 clocks
    ccdr.peripheral.DMA2.reset().enable();
    let dma = dp.DMA2;

    // The DMAMUX is used to connect DMA requests from peripherals, etc. to specific DMA channels.
    let channel = 9; // DMAMUX channel 9 <-> DMA2 channel 1
    dp.DMAMUX1.ccr[channel].write(|w| w.dmareq_id().usart3_tx_dma());
    dp.DMAMUX1.cfr.write(|w| w.csof9().set_bit()); // Clear synchro overrun flag

    // Enable DMA interrupt
    unsafe { pac::NVIC::unmask(pac::Interrupt::DMA2_STR1); }

    let (prod, cons) = unsafe { QUEUE.try_split().unwrap() };
    cortex_m::interrupt::free(|cs| {
        PROD.borrow(cs).borrow_mut().replace(prod);
        DMA.borrow(cs).borrow_mut().replace(DmaState {
            dma,
            uart,
            cons,
            grant: None,
        });
    });

    info!("Starting DMA write...         ");

    let mut x = 0u32;
    loop {
        // Write something to test the DMA
        write!(Writer(), "{}\t", x).unwrap();
        write!(Writer(), "{}\t", x + 1).unwrap();
        write!(Writer(), "{}\t", x + 3).unwrap();
        write!(Writer(), "{}\t", x + 4).unwrap();
        write!(Writer(), "{}\t", x + 5).unwrap();
        write!(Writer(), "{}\t", x + 6).unwrap();
        writeln!(Writer(), "{}", x + 7).unwrap();
        x += 8;
    }
}

#[pac::interrupt]
fn DMA2_STR1() {
    static mut DMA2_STR1_ERR: usize = 0;

    cortex_m::interrupt::free(|cs| {
        let mut rc = DMA.borrow(cs).borrow_mut();
        let mut state = rc.take().unwrap();

        // cache interrupt state register
        let isr = state.dma.lisr.read();

        // clear interrupt flags
        state.dma.lifcr.write(|w| {
            w.ctcif1().clear()
                .chtif1().clear()
                .cteif1().clear()
                .cdmeif1().clear()
                .cfeif1().clear()
        });

        // handle interrupt events
        if isr.tcif1().is_complete() {
            state.complete_dma();
            state.start_dma_tx();
        }

        rc.replace(state);

        // Record unhandled errors
        if isr.dmeif1().is_error() {
            *DMA2_STR1_ERR += 1;
        }

        if isr.teif1().is_error() {
            *DMA2_STR1_ERR += 1;
        }

        if isr.feif1().is_error() {
            *DMA2_STR1_ERR += 1;
        }
    });
}
