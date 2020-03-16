//! Testing PWM output for pre-defined pin combination: all pins for default mapping

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use stm32f1xx_hal::{
    prelude::*,
    pac,
    timer::{Tim3FullRemap, Timer},
    time::U32Ext,
    capture::{Channel, Capture, CaptureExt, ActiveTransition}
};
use cortex_m_rt::entry;

use nb::block;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    // let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

    // TIM2
    // let c1 = gpioa.pa0.into_floating_input(&mut gpioa.crl);
    // let c2 = gpioa.pa1.into_floating_input(&mut gpioa.crl);
    // let c3 = gpioa.pa2.into_floating_input(&mut gpioa.crl);
    // If you don't want to use all channels, just leave some out
    // let c4 = gpioa.pa3.into_floating_input(&mut gpioa.crl);

    // TIM3
    // let c1 = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    // let c2 = gpioa.pa7.into_floating_input(&mut gpioa.crl);
    // let c3 = gpiob.pb0.into_floating_input(&mut gpiob.crl);
    // let c4 = gpiob.pb1.into_floating_input(&mut gpiob.crl);
    let c2 = gpioc.pc7.into_floating_input(&mut gpioc.crl);

    // TIM4 (Only available with the "medium" density feature)
    // let c1 = gpiob.pb6.into_floating_input(&mut gpiob.crl);
    // let c2 = gpiob.pb7.into_floating_input(&mut gpiob.crl);
    // let c3 = gpiob.pb8.into_floating_input(&mut gpiob.crh);
    // let c4 = gpiob.pb9.into_floating_input(&mut gpiob.crh);

    let pins = c2;

    let mut capture = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1)
        .capture::<Tim3FullRemap, _, _>(pins, &mut afio.mapr);

    //// Operations affecting all defined channels on the Timer

    // Adjust period to 10 khz
    capture.set_resolution(10.khz() );
    capture.set_transition(ActiveTransition::RisingEdge, Channel::C1).unwrap();

    asm::bkpt();

    let before = block!(capture.capture(Channel::C1)).unwrap();
    let after = block!(capture.capture(Channel::C1)).unwrap();

    asm::bkpt();

    let period = after.wrapping_sub(before);

    //println!("Period: {} ms", period);

    asm::bkpt();

    loop {}
}
