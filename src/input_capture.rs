//! ```
//! use stm32f1xx_hal::stm32::TIM3;
//! use stm32f1xx_hal::gpio::gpiob::{PB4, PB5};
//! use stm32f1xx_hal::gpio::{Input, Floating};
//! use stm32f1xx_hal::pwm::{Pins, Pwm, C1, C2, C3, C4};
//!
//! struct MyChannels(PB4<Input<Floating>>,  PB5<Input<Floating>>);
//!
//! impl Pins<TIM3> for MyChannels
//! {
//! const REMAP: u8 = 0b10;
//! const C1: bool = true;
//! const C2: bool = true;
//! const C3: bool = false;
//! const C4: bool = false;
//! type Channels = (Pwm<TIM3, C1>, Pwm<TIM3, C2>)
//! }
//!
//! ...
//!
//! let gpiob = ..; // Set up and split GPIOB
//!
//! let p1 = gpiob.pb4.into_alternate_push_pull(&mut gpiob.crl);
//! let p2 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);
//!
//! ...
//!
//! let device: pac::Peripherals = ..;
//!
//! let (c1, c2) = device.TIM3.pwm(
//! MyChannels(p1, p2),
//! &mut afio.mapr,
//! 100.hz(),
//! clocks,
//! &mut rcc.apb1
//! );
//!
//! Set the duty cycle of channel 0 to 50%
//! c1.set_duty(c1.get_max_duty() / 2);
//! PWM outputs are disabled by default
//! c1.enable()
//! ```

use core::{marker::PhantomData, mem};

use crate::{
    afio::MAPR,
    bb,
    gpio::{
        gpioa::{PA0, PA1, PA2, PA3, PA6, PA7},
        gpiob::{PB0, PB1, PB6, PB7, PB8, PB9},
        Input, Floating,
    },
    rcc::{Clocks, APB1},
    time::{Hertz, U32Ext},
    timer::PclkSrc,
    pac::{
        tim2::{CCMR1_INPUT, CCMR2_INPUT},
        TIM2, TIM3, TIM4,
    },
};

pub trait InputCaptureExt {
    #[allow(unsafe_code)]
    unsafe fn ccmr1_input(&self) -> &CCMR1_INPUT;
    #[allow(unsafe_code)]
    unsafe fn ccmr2_input(&self) -> &CCMR2_INPUT;
}
impl InputCaptureExt for crate::pac::tim2::RegisterBlock {
    #[allow(unsafe_code)]
    unsafe fn ccmr1_input(&self) -> &CCMR1_INPUT {
        let ptr: *const _ = &self.ccmr1_output();
        &*(ptr as *const CCMR1_INPUT)
    }

    #[allow(unsafe_code)]
    unsafe fn ccmr2_input(&self) -> &CCMR2_INPUT {
        let ptr: *const _ = &self.ccmr2_output();
        &*(ptr as *const CCMR2_INPUT)
    }
}

pub trait Pins<TIM> {
    const REMAP: u8;
    const C1: bool;
    const C2: bool;
    const C3: bool;
    const C4: bool;
    type Channels;
}

impl Pins<TIM2>
    for (PA0<Input<Floating>>, PA1<Input<Floating>>, PA2<Input<Floating>>, PA3<Input<Floating>>)
{
    #[allow(clippy::type_complexity)]
    type Channels = (InputChannel<TIM2, C1>, InputChannel<TIM2, C2>, InputChannel<TIM2, C3>, InputChannel<TIM2, C4>);

    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    const REMAP: u8 = 0b00;
}

impl Pins<TIM3>
    for (PA6<Input<Floating>>, PA7<Input<Floating>>, PB0<Input<Floating>>, PB1<Input<Floating>>)
{
    #[allow(clippy::type_complexity)]
    type Channels = (InputChannel<TIM3, C1>, InputChannel<TIM3, C2>, InputChannel<TIM3, C3>, InputChannel<TIM3, C4>);

    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    const REMAP: u8 = 0b00;
}

impl Pins<TIM4>
    for (PB6<Input<Floating>>, PB7<Input<Floating>>, PB8<Input<Floating>>, PB9<Input<Floating>>)
{
    #[allow(clippy::type_complexity)]
    type Channels = (InputChannel<TIM4, C1>, InputChannel<TIM4, C2>, InputChannel<TIM4, C3>, InputChannel<TIM4, C4>);

    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    const REMAP: u8 = 0b0;
}

pub trait InputChannelExt: Sized {
    fn input_channel<PINS, T>(self, _: PINS, freq: T, clocks: Clocks, mapr: &mut MAPR, apb: &mut APB1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>;
}

impl InputChannelExt for TIM2 {
    fn input_channel<PINS, T>(self, pins: PINS, freq: T, clocks: Clocks, mapr: &mut MAPR, apb: &mut APB1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        #[allow(unsafe_code)]
        unsafe {
            mapr.mapr().modify(|_, w| w.tim2_remap().bits(PINS::REMAP));
        }

        tim2(self, pins, freq.into(), clocks, apb)
    }
}

impl InputChannelExt for TIM3 {
    fn input_channel<PINS, T>(self, pins: PINS, freq: T, clocks: Clocks, mapr: &mut MAPR, apb: &mut APB1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        #[allow(unsafe_code)]
        unsafe {
            mapr.mapr().modify(|_, w| w.tim3_remap().bits(PINS::REMAP));
        }

        tim3(self, pins, freq.into(), clocks, apb)
    }
}

impl InputChannelExt for TIM4 {
    fn input_channel<PINS, T>(self, pins: PINS, freq: T, clocks: Clocks, mapr: &mut MAPR, apb: &mut APB1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        mapr.mapr().modify(|_, w| w.tim4_remap().bit(PINS::REMAP == 1));

        tim4(self, pins, freq.into(), clocks, apb)
    }
}

pub struct InputChannel<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

#[allow(clippy::pub_enum_variant_names)]
#[allow(dead_code)]
pub enum Prescaler {
    Div1 = 0b00,
    Div2 = 0b01,
    Div4 = 0b10,
    Div8 = 0b11,
}
impl Prescaler {
    pub fn from_register(setting: u8) -> Self {
        match setting {
            0b00 => Prescaler::Div1,
            0b01 => Prescaler::Div2,
            0b10 => Prescaler::Div4,
            0b11 => Prescaler::Div8,
            _    => Prescaler::Div1,
        }
    }
    pub fn to_cnt(self) -> u8 {
        match self {
            Prescaler::Div1 => 1,
            Prescaler::Div2 => 2,
            Prescaler::Div4 => 4,
            Prescaler::Div8 => 8,
        }
    }
}

#[allow(dead_code)]
pub enum Filter {
    CkInt = 0b0000,
    CkInt2 = 0b0001,
    CkInt4 = 0b0010,
    CkInt8 = 0b0011,
    Dts2_6 = 0b0100,
    Dts2_8 = 0b0101,
    Dts4_6 = 0b0110,
    Dts4_8 = 0b0111,
    Dts8_6 = 0b1000,
    Dts8_8 = 0b1001,
    Dts16_5 = 0b1010,
    Dts16_6 = 0b1011,
    Dts16_8 = 0b1100,
    Dts32_5 = 0b1101,
    Dts32_6 = 0b1110,
    Dts32_8 = 0b1111,
}

#[allow(dead_code)]
pub enum ActiveTransition {
    RisingEdge = 0,
    FallingEdge = 1,
}

pub trait InputChannelPin {
    fn disable(&mut self);
    fn enable(&mut self);

    fn listen(&mut self);
    fn unlisten(&mut self);
    fn generate_event(&mut self);

    fn get_herz(&mut self, clocks: Clocks) -> Hertz;
    fn has_overcapture(&self) -> bool;
    fn has_event(&self) -> bool;

    fn set_prescaler(&mut self, prescaler: Prescaler);
    fn set_filter(&mut self, filter: Filter);
    fn set_transition(&mut self, edge: ActiveTransition);
}

pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;

/// Courtesy of @TeXitoi (https://github.com/stm32-rs/stm32f1xx-hal/pull/10#discussion_r259535503)
fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u16) {
    if freq == 0 {
        return (core::u16::MAX, core::u16::MAX);
    }
    let presc = clock / freq.saturating_mul(core::u16::MAX as u32 + 1);
    let arr = clock / freq.saturating_mul(presc + 1);
    (core::cmp::max(1, arr as u16), presc as u16)
}

fn compute_freq(counter: u32, clock: u32, prescaler: Prescaler, presc: u32) -> Hertz {
    let counter = core::cmp::max(1, counter / prescaler.to_cnt() as u32);
    ( (clock / presc) / counter).hz()
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            fn $timX<PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                clocks: Clocks,
                apb: &mut APB1,
            ) -> PINS::Channels
            where
                PINS: Pins<$TIMX>,
            {
                apb.enr().modify(|_, w| w.$timXen().set_bit());
                apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                if PINS::C1 {
                    #[allow(unsafe_code)]
                    unsafe {
                        tim.ccmr1_input().modify(|_, w|
                            w.cc1s().bits(0b01)
                        );
                    }
                }

                if PINS::C2 {
                    #[allow(unsafe_code)]
                    unsafe {
                        tim.ccmr1_input().modify(|_, w|
                            w.cc2s().bits(0b01)
                        );
                    }
                }

                if PINS::C3 {
                    #[allow(unsafe_code)]
                    unsafe {
                        tim.ccmr2_input().modify(|_, w|
                            w.cc3s().bits(0b01)
                        );
                    }
                }

                if PINS::C4 {
                    #[allow(unsafe_code)]
                    unsafe {
                        tim.ccmr2_input().modify(|_, w|
                            w.cc4s().bits(0b01)
                        );
                    }
                }

                let clk = $TIMX::get_clk(&clocks).0;
                #[allow(unsafe_code)]
                unsafe {
                    let (arr, psc) = compute_arr_presc(freq.0, clk);
                    tim.psc.write(|w| w.psc().bits(psc) );
                    tim.arr.write(|w| { w.arr().bits(arr) });

                    tim.cr1.write(|w|
                        w.cms()
                            .bits(0b00)
                            .dir()
                            .clear_bit()
                            .opm()
                            .clear_bit()
                            .cen()
                            .set_bit()
                    );
                }

                #[allow(unsafe_code)]
                unsafe { mem::uninitialized() }
            }

            impl InputChannelPin for InputChannel<$TIMX, C1> {
                fn disable(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc1e().clear_bit() ) }
                }

                fn enable(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc1e().set_bit() ) }
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc1if().clear_bit() ) }
                }

                fn listen(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc1ie().set_bit() ) }
                }

                fn unlisten(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc1ie().clear_bit() ) }
                }

                fn generate_event(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).egr.write(|w| w.cc1g().set_bit() ) }
                }

                fn get_herz(&mut self, clocks: Clocks) -> Hertz {
                    #[allow(unsafe_code)]
                    unsafe {
                        let current = (*$TIMX::ptr()).ccr1.read().ccr().bits() as u32;

                        // Reset Counter
                        (*$TIMX::ptr()).cnt.reset();

                        let clk = $TIMX::get_clk(&clocks).0;
                        let prescaler = Prescaler::from_register( (*$TIMX::ptr()).ccmr1_input().read().ic1psc().bits() as u8);
                        let psc = (*$TIMX::ptr()).psc.read().psc().bits() as u32;

                        compute_freq(current, clk, prescaler, psc)
                    }
                }

                fn has_overcapture(&self) -> bool {
                    #[allow(unsafe_code)]
                    let ret = unsafe { (*$TIMX::ptr()).sr.read().cc1of().bit_is_set() };
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc1of().clear_bit() ) };

                    ret
                }

                fn has_event(&self) -> bool {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.read().cc1if().bit_is_set() }
                }

                fn set_prescaler(&mut self, prescaler: Prescaler) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccmr1_input().modify(|_, w| w.ic1psc().bits(prescaler as u8)) }
                }
                fn set_filter(&mut self, filter: Filter) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccmr1_input().modify(|_, w| w.ic1f().bits(filter as u8)) }
                }
                fn set_transition(&mut self, edge: ActiveTransition) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc1p().bit(edge as u8 != 0)) }
                }
            }

            impl InputChannelPin for InputChannel<$TIMX, C2> {
                fn disable(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc2e().clear_bit() ) }
                }

                fn enable(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc2e().set_bit() ) }
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc2if().clear_bit() ) }
                }

                fn listen(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc2ie().set_bit() ) }
                }

                fn unlisten(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc2ie().clear_bit() ) }
                }

                fn generate_event(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).egr.write(|w| w.cc2g().set_bit() ) }
                }

                fn get_herz(&mut self, clocks: Clocks) -> Hertz {
                    #[allow(unsafe_code)]
                    unsafe {
                        let current = (*$TIMX::ptr()).ccr2.read().ccr().bits() as u32;

                        // Reset Counter
                        (*$TIMX::ptr()).cnt.reset();

                        let clk = $TIMX::get_clk(&clocks).0;
                        let prescaler = Prescaler::from_register( (*$TIMX::ptr()).ccmr1_input().read().ic2psc().bits() as u8);
                        let psc = (*$TIMX::ptr()).psc.read().psc().bits() as u32;

                        compute_freq(current, clk, prescaler, psc)
                    }
                }

                fn has_overcapture(&self) -> bool {
                    #[allow(unsafe_code)]
                    let ret = unsafe { (*$TIMX::ptr()).sr.read().cc2of().bit_is_set() };
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc2of().clear_bit() ) };

                    ret
                }

                fn has_event(&self) -> bool {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.read().cc2if().bit_is_set() }
                }

                fn set_prescaler(&mut self, prescaler: Prescaler) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccmr1_input().modify(|_, w| w.ic2psc().bits(prescaler as u8)) }
                }
                fn set_filter(&mut self, filter: Filter) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccmr1_input().modify(|_, w| w.ic2f().bits(filter as u8)) }
                }
                fn set_transition(&mut self, edge: ActiveTransition) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc2p().bit(edge as u8 != 0)) }
                }
            }

            impl InputChannelPin for InputChannel<$TIMX, C3> {
                fn disable(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc3e().clear_bit() ) }
                }

                fn enable(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc3e().set_bit() ) }
                }

                fn listen(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc3ie().set_bit() ) }
                }

                fn unlisten(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc3ie().clear_bit() ) }
                }

                fn generate_event(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).egr.write(|w| w.cc3g().set_bit() ) }
                }

                fn get_herz(&mut self, clocks: Clocks) -> Hertz {
                    #[allow(unsafe_code)]
                    unsafe {
                        let current = (*$TIMX::ptr()).ccr3.read().ccr().bits() as u32;

                        // Reset Counter
                        (*$TIMX::ptr()).cnt.reset();

                        let clk = $TIMX::get_clk(&clocks).0;
                        let prescaler = Prescaler::from_register( (*$TIMX::ptr()).ccmr2_input().read().ic3psc().bits() as u8);
                        let psc = (*$TIMX::ptr()).psc.read().psc().bits() as u32;

                        compute_freq(current, clk, prescaler, psc)
                    }
                }

                fn has_overcapture(&self) -> bool {
                    #[allow(unsafe_code)]
                    let ret = unsafe { (*$TIMX::ptr()).sr.read().cc3of().bit_is_set() };
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc3of().clear_bit() ) };

                    ret
                }

                fn has_event(&self) -> bool {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.read().cc3if().bit_is_set() }
                }

                fn set_prescaler(&mut self, prescaler: Prescaler) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccmr2_input().modify(|_, w| w.ic3psc().bits(prescaler as u8)) }
                }
                fn set_filter(&mut self, filter: Filter) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccmr2_input().modify(|_, w| w.ic3f().bits(filter as u8)) }
                }
                fn set_transition(&mut self, edge: ActiveTransition) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc3p().bit(edge as u8 != 0)) }
                }
            }

            impl InputChannelPin for InputChannel<$TIMX, C4> {
                fn disable(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc4e().clear_bit() ) }
                }

                fn enable(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc4e().set_bit() ) }
                }

                fn listen(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc4ie().set_bit() ) }
                }

                fn unlisten(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc4ie().clear_bit() ) }
                }

                fn generate_event(&mut self) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).egr.write(|w| w.cc4g().set_bit() ) }
                }

                fn get_herz(&mut self, clocks: Clocks) -> Hertz {
                    #[allow(unsafe_code)]
                    unsafe {
                        let current = (*$TIMX::ptr()).ccr4.read().ccr().bits() as u32;

                        // Reset Counter
                        (*$TIMX::ptr()).cnt.reset();

                        let clk = $TIMX::get_clk(&clocks).0;
                        let prescaler = Prescaler::from_register( (*$TIMX::ptr()).ccmr2_input().read().ic4psc().bits() as u8);
                        let psc = (*$TIMX::ptr()).psc.read().psc().bits() as u32;

                        compute_freq(current, clk, prescaler, psc)
                    }
                }

                fn has_overcapture(&self) -> bool {
                    #[allow(unsafe_code)]
                    let ret = unsafe { (*$TIMX::ptr()).sr.read().cc4of().bit_is_set() };
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc4of().clear_bit() ) };

                    ret
                }

                fn has_event(&self) -> bool {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).sr.read().cc4if().bit_is_set() }
                }

                fn set_prescaler(&mut self, prescaler: Prescaler) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccmr2_input().modify(|_, w| w.ic4psc().bits(prescaler as u8)) }
                }
                fn set_filter(&mut self, filter: Filter) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccmr2_input().modify(|_, w| w.ic4f().bits(filter as u8)) }
                }
                fn set_transition(&mut self, edge: ActiveTransition) {
                    #[allow(unsafe_code)]
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc4p().bit(edge as u8 != 0)) }
                }
            }
        )+
    }
}

hal! {
    TIM2: (tim2, tim2en, tim2rst),
    TIM3: (tim3, tim3en, tim3rst),
    TIM4: (tim4, tim4en, tim4rst),
}
