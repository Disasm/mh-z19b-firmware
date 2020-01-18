#![no_main]
#![no_std]

use panic_halt as _;

use stm32f0xx_hal::{prelude::*, stm32, stm32::{interrupt, DMA1, ADC}};
use stm32f0xx_hal::serial::Serial;
use stm32f0xx_hal::timers::{Timer, Event};
use cortex_m_rt::entry;
use core::fmt::Write;
use stm32f0xx_hal::delay::Delay;
use core::sync::atomic::{AtomicBool, Ordering};

const ADC_ITEM_COUNT: usize = 80;
static mut ADC_DMA_BUF: [u16; 4] = [0; 4];

static mut ADC_LIGHT: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
static mut ADC_TEMP: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
static mut ADC_VREF: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
static ADC_DATA_READY: AtomicBool = AtomicBool::new(false);

#[interrupt]
fn DMA1_CH1() {
    // Warning: macro-magic happens here
    static mut INDEX: usize = 0;
    static mut BUF_LIGHT: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
    static mut BUF_TEMP: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];
    static mut BUF_VREF: [u16; ADC_ITEM_COUNT] = [0; ADC_ITEM_COUNT];

    let dma = unsafe { &*DMA1::ptr() };
    let adc = unsafe { &*ADC::ptr() };

    if dma.isr.read().tcif1().is_complete() {
        // Stop conversion
        adc.cr.modify(|_, w| w.adstp().set_bit());

        // Clear TCIF flag
        dma.ifcr.write_with_zero(|w| w.ctcif1().set_bit());

        let (light, temp, vref) = unsafe {
            (ADC_DMA_BUF[2], ADC_DMA_BUF[0], ADC_DMA_BUF[3])
        };

        let index = *INDEX;
        if index < ADC_ITEM_COUNT {
            BUF_LIGHT[index] = light;
            BUF_TEMP[index] = temp;
            BUF_VREF[index] = vref;

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

fn process_data(mut serial: impl Write) {
    if !ADC_DATA_READY.load(Ordering::SeqCst) {
        return;
    }
    let (adc_light, adc_temp, adc_vref) = unsafe {
        (ADC_LIGHT, ADC_TEMP, ADC_VREF)
    };
    ADC_DATA_READY.store(false, Ordering::SeqCst);

    // TODO: process data
    write!(serial, "ADC data: {} {} {}", adc_light[0], adc_temp[0], adc_vref[0]).unwrap();
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH;
    let mut rcc = dp.RCC.configure().sysclk(8.mhz()).freeze(&mut flash);

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

    let mut tim2 = Timer::tim2(dp.TIM2, 1.khz(), &mut rcc);
    tim2.listen(Event::TimeOut);

    let mut delay = Delay::new(cp.SYST, &rcc);

    writeln!(serial, "Hello, world!\r").unwrap();

    loop {
        process_data(&mut serial);
        delay.delay_ms(3_000u32);
        lamp_pin.set_high().unwrap();
        delay.delay_ms(400u32);
        lamp_pin.set_low().unwrap();
    }
}
