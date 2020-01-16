#![no_main]
#![no_std]

use panic_halt as _;

use stm32f0xx_hal::{prelude::*, stm32};
use stm32f0xx_hal::serial::Serial;
use stm32f0xx_hal::timers::{Timer, Event};
use cortex_m_rt::entry;
use core::fmt::Write;
use stm32f0xx_hal::delay::Delay;

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
        delay.delay_ms(3_000u32);
        lamp_pin.set_high().unwrap();
        delay.delay_ms(400u32);
        lamp_pin.set_low().unwrap();
    }
}
