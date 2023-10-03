#![no_main]
#![no_std]

use panic_halt as _;

use stm32f0xx_hal::{prelude::*, stm32, stm32::interrupt};
use stm32f0xx_hal::stm32::{ADC, DMA1, RCC};
use stm32f0xx_hal::serial::Serial;
use stm32f0xx_hal::timers::{Timer, Event};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering, AtomicU32};

const ADC_ITEM_COUNT: usize = 80;
static mut ADC_DMA_BUF: [u16; 20] = [0; 20];

static mut ADC_LIGHT: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
static mut ADC_TEMP: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
static mut ADC_VREF: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
static ADC_DATA_READY: AtomicBool = AtomicBool::new(false);

/// Start ADC conversion
fn adc_start() {
    let adc = unsafe { &*ADC::ptr() };
    adc.cr.modify(|_, w| w.adstart().set_bit());
}

/// Stop ADC conversion
fn adc_stop() {
    let adc = unsafe { &*ADC::ptr() };
    adc.cr.modify(|_, w| w.adstp().set_bit());
}

#[interrupt]
fn DMA1_CH1() {
    // Warning: macro-magic happens here
    static mut INDEX: usize = 0;
    static mut BUF_LIGHT: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
    static mut BUF_TEMP: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
    static mut BUF_VREF: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];

    let dma = unsafe { &*DMA1::ptr() };
    if dma.isr.read().tcif1().is_complete() {
        adc_stop();

        // Clear TCIF flag
        unsafe { dma.ifcr.write_with_zero(|w| w.ctcif1().set_bit()) };

        let data = unsafe {
            if ADC_DMA_BUF.len() > 4 {
                let n = ADC_DMA_BUF.len() / 4;
                let mut sum = [0u32; 4];
                for i in 0..n {
                    sum[0] += u32::from(ADC_DMA_BUF[i * 4]);
                    sum[1] += u32::from(ADC_DMA_BUF[i * 4 + 1]);
                    sum[2] += u32::from(ADC_DMA_BUF[i * 4 + 2]);
                    sum[3] += u32::from(ADC_DMA_BUF[i * 4 + 3]);
                }

                [
                    (sum[0] / (n as u32)) as u16,
                    (sum[1] / (n as u32)) as u16,
                    (sum[2] / (n as u32)) as u16,
                    (sum[3] / (n as u32)) as u16,
                ]
            } else {
                [
                    ADC_DMA_BUF[0],
                    ADC_DMA_BUF[1],
                    ADC_DMA_BUF[2],
                    ADC_DMA_BUF[3],
                ]
            }
        };

        let index = *INDEX;
        if index < ADC_ITEM_COUNT {
            BUF_LIGHT[index] = data[2];
            BUF_TEMP[index] = data[0];
            BUF_VREF[index] = data[3];

            if index == (ADC_ITEM_COUNT - 1) {
                *INDEX = 0;

                if !ADC_DATA_READY.load(Ordering::SeqCst) {
                    unsafe {
                        ADC_LIGHT.copy_from_slice(BUF_LIGHT);
                        ADC_TEMP.copy_from_slice(BUF_TEMP);
                        ADC_VREF.copy_from_slice(BUF_VREF);
                    }
                    ADC_DATA_READY.store(true, Ordering::SeqCst);
                }
            } else {
                *INDEX = index + 1;
            }
        }
    }
}

static TIM2_CNT: AtomicU32 = AtomicU32::new(0);

#[interrupt]
fn TIM2() {
    static mut SAMPLE_TICK: usize = 0;

    let tim = unsafe { &*stm32::TIM2::ptr() };
    tim.sr.modify(|_, w| w.uif().clear_bit());

    let mut tick = TIM2_CNT.load(Ordering::SeqCst);

    let gpiob = unsafe { &*stm32::GPIOB::ptr() };
    if tick == 100 {
        gpiob.bsrr.write(|w| w.bs1().set_bit());
    }
    if tick == 500 {
        gpiob.bsrr.write(|w| w.br1().set_bit());
    }

    if tick as usize == *SAMPLE_TICK * 10 && *SAMPLE_TICK < ADC_ITEM_COUNT {
        adc_start();
        *SAMPLE_TICK += 1;
    }

    tick += 1;
    if tick == 4000 {
        tick = 0;
        *SAMPLE_TICK = 0;
    }
    TIM2_CNT.store(tick, Ordering::SeqCst);
}

unsafe fn setup_adc(adc: ADC, dma: DMA1, nvic: &mut NVIC) {
    let rcc = unsafe { &*RCC::ptr() };
    rcc.apb2enr.modify(|_, w| w.adcen().set_bit());
    rcc.ahbenr.modify(|_, w| w.dmaen().set_bit());

    // Reset ADC
    rcc.apb2rstr.modify(|_, w| w.adcrst().set_bit());
    rcc.apb2rstr.modify(|_, w| w.adcrst().clear_bit());

    let adc_dr_addr = &adc.dr as *const _ as usize;
    let dma_buf_addr = unsafe { ADC_DMA_BUF.as_ptr() as usize };
    let dma_buf_count = unsafe { ADC_DMA_BUF.len() };

    let ch = &dma.ch1;
    // deinit
    ch.cr.modify(|_, w| w.en().clear_bit());
    ch.cr.write_with_zero(|w| w);
    ch.ndtr.write_with_zero(|w| w);
    ch.par.write_with_zero(|w| w);
    ch.mar.write_with_zero(|w| w);
    dma.ifcr.write_with_zero(|w| {
        w.cteif1().set_bit();
        w.chtif1().set_bit();
        w.ctcif1().set_bit();
        w.cgif1().set_bit();
        w
    });

    ch.cr.write_with_zero(|w| {
        w.mem2mem().disabled();
        w.pl().very_high();
        w.msize().bits16();
        w.psize().bits16();
        w.minc().enabled();
        w.pinc().disabled();
        w.circ().enabled();
        w.dir().from_peripheral();
        w.teie().disabled();
        w.htie().disabled();
        w.tcie().disabled();
        w.en().clear_bit();
        w
    });
    ch.ndtr.write_with_zero(|w| w.ndt().bits(dma_buf_count as u16));
    ch.par.write_with_zero(|w| w.pa().bits(adc_dr_addr as u32));
    ch.mar.write_with_zero(|w| w.ma().bits(dma_buf_addr as u32));
    ch.cr.modify(|_, w| {
        w.tcie().set_bit();  // Enable "transfer complete" interrupt
        w.en().set_bit();  // Enable channel
        w
    });

    // Select DMA circular mode
    adc.cfgr1.modify(|_, w| w.dmacfg().set_bit());

    // Configure ADC
    adc.cfgr1.modify(|_, w| unsafe {
        w.cont().continuous();
        w.exten().disabled();
        w.extsel().bits(0);
        w.align().right();
        w.res().twelve_bit();
        w.scandir().backward();
        w
    });

    // Enable ADC channels and set sampling time
    adc.chselr.modify(|_, w| {
        w.chsel1().set_bit(); // PA1 - Vref
        w.chsel2().set_bit(); // PA2 - sensor (light)
        w.chsel3().set_bit(); // NC
        w.chsel6().set_bit(); // PA6 - sensor (temp)
        w
    });
    adc.smpr.write(|w| w.smp().cycles239_5());

    // Calibrate ADC
    adc.cr.modify(|_, w| w.adcal().set_bit());
    for _ in 0..0xf000 {
        if adc.cr.read().adcal().bit_is_clear() { break }
    }

    // Enable ADC
    adc.cr.modify(|_, w| w.aden().set_bit());
    // Wait for ADC to be ready
    while adc.isr.read().adrdy().bit_is_clear() {}

    // Enable DMA requests
    adc.cfgr1.modify(|_, w| w.dmaen().set_bit());

    // Enable DMA interrupt
    unsafe {
        use stm32::Interrupt::DMA1_CH1;
        nvic.set_priority(DMA1_CH1, 0);
        NVIC::unmask(DMA1_CH1);
    }
}

fn process_data(mut serial: impl Write) {
    if !ADC_DATA_READY.load(Ordering::SeqCst) {
        return;
    }
    let (adc_light, adc_temp, adc_vref) = unsafe {
        (ADC_LIGHT, ADC_TEMP, ADC_VREF)
    };
    ADC_DATA_READY.store(false, Ordering::SeqCst);

    write!(serial, "\r\n").ok();
    write!(serial, "LIGHT:").ok();
    for b in adc_light.iter() {
        write!(serial, " {:02x}", *b).ok();
    }
    write!(serial, "\r\n").ok();

    write!(serial, "TEMP: ").ok();
    for b in adc_temp.iter() {
        write!(serial, " {:02x}", *b).ok();
    }
    write!(serial, "\r\n").ok();

    write!(serial, "VREF: ").ok();
    for b in adc_vref.iter() {
        write!(serial, " {:02x}", *b).ok();
    }
    write!(serial, "\r\n").ok();

    // TODO: process data
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH;
    let mut rcc = dp.RCC.configure().sysclk(8.mhz()).freeze(&mut flash);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // Configure USART
    let (usart1, pb6, pb7) = (dp.USART1, gpiob.pb6, gpiob.pb7);
    let mut serial = cortex_m::interrupt::free(|cs| {
        let tx = pb6.into_alternate_af0(cs);
        let rx = pb7.into_alternate_af0(cs);
        Serial::usart1(usart1, (tx, rx), 115_200.bps(), &mut rcc)
    });

    // Configure lamp output
    let pb1 = gpiob.pb1;
    let mut lamp_pin = cortex_m::interrupt::free(|cs| {
        // (Re-)configure PA1 as output
        pb1.into_push_pull_output(cs)
    });
    lamp_pin.set_low().unwrap();

    writeln!(serial, "Hello, world!\r").unwrap();

    let (_pa1, _pa2, _pa6) = cortex_m::interrupt::free(|cs| (
        gpioa.pa1.into_analog(cs),
        gpioa.pa2.into_analog(cs),
        gpioa.pa6.into_analog(cs),
    ));
    unsafe { setup_adc(dp.ADC, dp.DMA1, &mut cp.NVIC) };

    let mut tim2 = Timer::tim2(dp.TIM2, 1.khz(), &mut rcc);
    tim2.listen(Event::TimeOut);
    unsafe { NVIC::unmask(stm32::Interrupt::TIM2); }

    loop {
        process_data(&mut serial);
    }
}
