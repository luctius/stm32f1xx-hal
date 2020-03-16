/*!
*/

use core::marker::PhantomData;
use core::marker::{Copy};

use cast::{u16, u32};
use crate::hal;
#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
use crate::pac::TIM1;
use crate::pac::{TIM2, TIM3};
#[cfg(feature = "medium")]
use crate::pac::TIM4;

use crate::afio::MAPR;
use crate::bb;
use crate::gpio::{self, Input, Floating};
use crate::time::Hertz;
use crate::time::U32Ext;
use crate::timer::Timer;

use embedded_hal::Capture as _;

#[derive(Clone, Debug, Copy, PartialEq, PartialOrd, Eq, Ord)]
pub enum CaptureError {
    OverCapture,
    UnavailableChannel,
}

#[derive(Clone, Debug, Copy, PartialEq, PartialOrd, Eq, Ord)]
pub enum Filter {
    CkInt   = 0b0000,
    CkInt2  = 0b0001,
    CkInt4  = 0b0010,
    CkInt8  = 0b0011,
    Dts2_6  = 0b0100,
    Dts2_8  = 0b0101,
    Dts4_6  = 0b0110,
    Dts4_8  = 0b0111,
    Dts8_6  = 0b1000,
    Dts8_8  = 0b1001,
    Dts16_5 = 0b1010,
    Dts16_6 = 0b1011,
    Dts16_8 = 0b1100,
    Dts32_5 = 0b1101,
    Dts32_6 = 0b1110,
    Dts32_8 = 0b1111,
}

#[derive(Clone, Debug, Copy, PartialEq, PartialOrd, Eq, Ord)]
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
}

#[derive(Clone, Debug, Copy, PartialEq, PartialOrd, Eq, Ord)]
pub enum ActiveTransition {
    RisingEdge  = 0,
    FallingEdge = 1,
}

pub trait Pins<REMAP, P> {
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels;

    fn is_used(c: Channel) -> bool {
        if (c == Channel::C1 && Self::C1)
            || (c == Channel::C2 && Self::C2)
            || (c == Channel::C3 && Self::C3)
            || (c == Channel::C4 && Self::C4)
        {
            true
        } else {
            false
        }
    }

    fn check_used(c: Channel) -> Channel {
        if Self::is_used(c) { c }
        else {
            panic!("Unused channel")
        }
    }
}

#[derive(Clone, Debug, Copy, PartialEq, PartialOrd, Eq, Ord)]
pub enum Channel {
    C1,
    C2,
    C3,
    C4,
}

use crate::timer::sealed::{Remap, Ch1, Ch2, Ch3, Ch4};
macro_rules! pins_impl {
    ( $( ( $($PINX:ident),+ ), ( $($TRAIT:ident),+ ), ( $($ENCHX:ident),* ); )+ ) => {
        $(
            #[allow(unused_parens)]
            impl<TIM, REMAP, $($PINX,)+> Pins<REMAP, ($($ENCHX),+)> for ($($PINX),+)
            where
                REMAP: Remap<Periph = TIM>,
                $($PINX: $TRAIT<REMAP> + gpio::Mode<Input<Floating>>,)+
            {
                $(const $ENCHX: bool = true;)+
                type Channels = ($(CaptureChannel<TIM, $ENCHX>),+);
            }
        )+
    };
}

pins_impl!(
    (P1, P2, P3, P4), (Ch1, Ch2, Ch3, Ch4), (C1, C2, C3, C4);
    (P2, P3, P4), (Ch2, Ch3, Ch4), (C2, C3, C4);
    (P1, P3, P4), (Ch1, Ch3, Ch4), (C1, C3, C4);
    (P1, P2, P4), (Ch1, Ch2, Ch4), (C1, C2, C4);
    (P1, P2, P3), (Ch1, Ch2, Ch3), (C1, C2, C3);
    (P3, P4), (Ch3, Ch4), (C3, C4);
    (P2, P4), (Ch2, Ch4), (C2, C4);
    (P2, P3), (Ch2, Ch3), (C2, C3);
    (P1, P4), (Ch1, Ch4), (C1, C4);
    (P1, P3), (Ch1, Ch3), (C1, C3);
    (P1, P2), (Ch1, Ch2), (C1, C2);
    (P1), (Ch1), (C1);
    (P2), (Ch2), (C2);
    (P3), (Ch3), (C3);
    (P4), (Ch4), (C4);
);

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
impl Timer<TIM1> {
    pub fn capture<REMAP, P, PINS>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
    ) -> Capture<TIM1, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM1>,
        PINS: Pins<REMAP, P>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim1_remap().bits(REMAP::REMAP) });

        let Self { tim, clk } = self;
        tim1(tim, _pins, clk)
    }
}

impl Timer<TIM2> {
    pub fn capture<REMAP, P, PINS>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
    ) -> Capture<TIM2, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM2>,
        PINS: Pins<REMAP, P>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim2_remap().bits(REMAP::REMAP) });

        let Self { tim, clk } = self;
        tim2(tim, _pins, clk)
    }
}

impl Timer<TIM3> {
    pub fn capture<REMAP, P, PINS>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
    ) -> Capture<TIM3, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM3>,
        PINS: Pins<REMAP, P>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim3_remap().bits(REMAP::REMAP) });

        let Self { tim, clk } = self;
        tim3(tim, _pins, clk)
    }
}

#[cfg(feature = "medium")]
impl Timer<TIM4> {
    pub fn capture<REMAP, P, PINS>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
    ) -> Capture<TIM4, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM4>,
        PINS: Pins<REMAP, P>,
    {
        mapr.modify_mapr(|_, w| w.tim4_remap().bit(REMAP::REMAP == 1));

        let Self { tim, clk } = self;
        tim4(tim, _pins, clk)
    }
}

pub struct Capture<TIM, REMAP, P, PINS>
where
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>
{
    clk: Hertz,
    _pins: PhantomData<(TIM, REMAP, P, PINS)>,
}

pub struct CaptureChannel<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

pub trait CaptureExt {
    type Error;
    type Channel;

    fn set_filter(&mut self, filter: Filter, channel: Channel) -> Result<(), Self::Error>;
    fn set_transition(&mut self, edge: ActiveTransition, channel: Channel) -> Result<(), Self::Error>;
    fn set_prescaler(&mut self, prescaler: Prescaler, channel: Channel) -> Result<(), Self::Error>;
    fn get_prescaler(&mut self, channel: Channel) -> Result<Prescaler, Self::Error>;
    fn generate_event(&mut self, channel: Channel) -> Result<(), Self::Error>;
    fn listen(&mut self, channel: Channel) -> Result<(), Self::Error>;
    fn unlisten(&mut self, channel: Channel) -> Result<(), Self::Error>;
}

pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident),)+) => {
        $(
            fn $timX<REMAP, P, PINS>(
                tim: $TIMX,
                _pins: PINS,
                clk: Hertz,
            ) -> Capture<$TIMX, REMAP, P, PINS>
            where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP, P>,
            {
                let mut c = Capture {
                    clk: clk,
                    _pins: PhantomData,
                };

                if PINS::C1 {
                    unsafe {
                        tim.ccmr1_input()
                            .modify(|_, w| w.cc1s().bits(0b01) );
                    }
                    let _ = c.set_prescaler(Prescaler::Div1, Channel::C1);
                    let _ = c.set_filter(Filter::CkInt, Channel::C1);
                    let _ = c.set_transition(ActiveTransition::RisingEdge, Channel::C1);
                }

                if PINS::C2 {
                    unsafe {
                        tim.ccmr1_input()
                            .modify(|_, w| w.cc2s().bits(0b01) );
                    }
                    let _ = c.set_prescaler(Prescaler::Div1, Channel::C2);
                    let _ = c.set_filter(Filter::CkInt, Channel::C2);
                    let _ = c.set_transition(ActiveTransition::RisingEdge, Channel::C2);
                }

                if PINS::C3 {
                    unsafe {
                        tim.ccmr2_input()
                            .modify(|_, w| w.cc3s().bits(0b01) );
                    }
                    let _ = c.set_prescaler(Prescaler::Div1, Channel::C3);
                    let _ = c.set_filter(Filter::CkInt, Channel::C3);
                    let _ = c.set_transition(ActiveTransition::RisingEdge, Channel::C3);
                }

                if PINS::C4 {
                    unsafe {
                        tim.ccmr2_input()
                            .modify(|_, w| w.cc4s().bits(0b01) );
                    }
                    let _ = c.set_prescaler(Prescaler::Div1, Channel::C4);
                    let _ = c.set_filter(Filter::CkInt, Channel::C4);
                    let _ = c.set_transition(ActiveTransition::RisingEdge, Channel::C4);
                }

                c.set_resolution(1.ms() );

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

                c
            }

        /*
        The following implemention of the embedded_hal::Capture uses Hertz as a time type.  This was choosen
        because of the timescales of operations being on the order of nanoseconds and not being able to
        efficently represent a float on the hardware.  It might be possible to change the time type to
        a different time based using such as the nanosecond.  The issue with doing so is that the max
        delay would then be at just a little over 2 seconds because of the 32 bit depth of the number.
        Using milliseconds is also an option, however, using this as a base unit means that only there
        could be resolution issues when trying to get a specific value, because of the integer nature.

        To find a middle ground, the Hertz type is used as a base here and the Into trait has been
        defined for several base time units.  This will allow for calling the set_period method with
        something that is natural to both the MCU and the end user.
        */
        impl<REMAP, P, PINS> hal::Capture for Capture<$TIMX, REMAP, P, PINS> where
            REMAP: Remap<Periph = $TIMX>,
            PINS: Pins<REMAP, P>,
            {
                type Error = CaptureError;
                type Channel = Channel;
                type Time = Hertz;
                type Capture = u16;

                fn enable(&mut self, channel: Self::Channel) {
                    match channel {
                        Channel::C1 => unsafe { bb::set(&(*$TIMX::ptr()).ccer, 0) },
                        Channel::C2 => unsafe { bb::set(&(*$TIMX::ptr()).ccer, 4) },
                        Channel::C3 => unsafe { bb::set(&(*$TIMX::ptr()).ccer, 8) },
                        Channel::C4 => unsafe { bb::set(&(*$TIMX::ptr()).ccer, 12) }
                    }
                }

                fn disable(&mut self, channel: Self::Channel) {
                    match channel {
                        Channel::C1 => unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 0) },
                        Channel::C2 => unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 4) },
                        Channel::C3 => unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 8) },
                        Channel::C4 => unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 12) },
                    }
                }

                fn capture(&mut self, channel: Self::Channel) -> nb::Result<Self::Capture, Self::Error> {
                    if !PINS::is_used(channel) {
                        return Err(nb::Error::Other(CaptureError::UnavailableChannel) );
                    }

                    let ready = match channel {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).sr.read().cc1if().bit_is_set() }
                        Channel::C2 => unsafe { (*$TIMX::ptr()).sr.read().cc2if().bit_is_set() }
                        Channel::C3 => unsafe { (*$TIMX::ptr()).sr.read().cc3if().bit_is_set() }
                        Channel::C4 => unsafe { (*$TIMX::ptr()).sr.read().cc4if().bit_is_set() }
                    };

                    let overcapture = match channel {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).sr.read().cc1of().bit_is_set() },
                        Channel::C2 => unsafe { (*$TIMX::ptr()).sr.read().cc2of().bit_is_set() },
                        Channel::C3 => unsafe { (*$TIMX::ptr()).sr.read().cc3of().bit_is_set() },
                        Channel::C4 => unsafe { (*$TIMX::ptr()).sr.read().cc4of().bit_is_set() }
                    };

                    if overcapture {
                        match channel {
                            Channel::C1 => unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc1of().clear_bit() ) },
                            Channel::C2 => unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc2of().clear_bit() ) },
                            Channel::C3 => unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc3of().clear_bit() ) },
                            Channel::C4 => unsafe { (*$TIMX::ptr()).sr.modify(|_, w| w.cc4of().clear_bit() ) },
                        }

                        Err(nb::Error::Other(CaptureError::OverCapture) )
                    }
                    else if !ready {
                        return Err(nb::Error::WouldBlock);
                    }
                    else {
                        Ok(match channel {
                            Channel::C1 => unsafe { (*$TIMX::ptr()).ccr1.read().ccr().bits() },
                            Channel::C2 => unsafe { (*$TIMX::ptr()).ccr2.read().ccr().bits() },
                            Channel::C3 => unsafe { (*$TIMX::ptr()).ccr3.read().ccr().bits() },
                            Channel::C4 => unsafe { (*$TIMX::ptr()).ccr4.read().ccr().bits() },
                        } as Self::Capture)
                    }
                }

                fn get_resolution(&self) -> Self::Time {
                    let clk = self.clk;
                    let psc: u16 = unsafe{(*$TIMX::ptr()).psc.read().psc().bits()};

                    // Number of clock ticks per capture tick
                    (clk.0 / u32(psc)).hz()
                }

                fn set_resolution<T>(&mut self, period: T) where
                    T: Into<Self::Time> {
                        let clk = self.clk;

                        let ticks = clk.0 / period.into().0;
                        let psc = u16(ticks).unwrap();
                        let arr = 0xFFFF_u16;

                        unsafe {
                            (*$TIMX::ptr()).psc.write(|w| w.psc().bits(psc));
                            (*$TIMX::ptr()).arr.write(|w| w.arr().bits(arr));
                        }
                }
            }

        impl<REMAP, P, PINS> CaptureExt for Capture<$TIMX, REMAP, P, PINS> where
            REMAP: Remap<Periph = $TIMX>,
            PINS: Pins<REMAP, P>,
            {
                type Error = CaptureError;
                type Channel = Channel;

                fn set_filter(&mut self, filter: Filter, channel: Channel) -> Result<(), Self::Error> {
                    if !PINS::is_used(channel) {
                        return Err(CaptureError::UnavailableChannel);
                    }

                    match channel {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).ccmr1_input().modify(|_, w| w.ic1f().bits(filter as u8)) },
                        Channel::C2 => unsafe { (*$TIMX::ptr()).ccmr1_input().modify(|_, w| w.ic2f().bits(filter as u8)) },
                        Channel::C3 => unsafe { (*$TIMX::ptr()).ccmr2_input().modify(|_, w| w.ic3f().bits(filter as u8)) },
                        Channel::C4 => unsafe { (*$TIMX::ptr()).ccmr2_input().modify(|_, w| w.ic4f().bits(filter as u8)) },
                    };
                    Ok( () )
                }
                fn set_transition(&mut self, edge: ActiveTransition, channel: Channel) -> Result<(), Self::Error> {
                    if !PINS::is_used(channel) {
                        return Err(CaptureError::UnavailableChannel);
                    }

                    match channel {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc3p().bit(edge as u8 != 0)) },
                        Channel::C2 => unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc3p().bit(edge as u8 != 0)) },
                        Channel::C3 => unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc3p().bit(edge as u8 != 0)) },
                        Channel::C4 => unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.cc3p().bit(edge as u8 != 0)) },
                    };
                    Ok( () )
                }
                fn set_prescaler(&mut self, prescaler: Prescaler, channel: Channel) -> Result<(), Self::Error> {
                    if !PINS::is_used(channel) {
                        return Err(CaptureError::UnavailableChannel);
                    }

                    match channel {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).ccmr1_input().modify(|_, w| w.ic1psc().bits(prescaler as u8)) }
                        Channel::C2 => unsafe { (*$TIMX::ptr()).ccmr1_input().modify(|_, w| w.ic2psc().bits(prescaler as u8)) }
                        Channel::C3 => unsafe { (*$TIMX::ptr()).ccmr2_input().modify(|_, w| w.ic3psc().bits(prescaler as u8)) }
                        Channel::C4 => unsafe { (*$TIMX::ptr()).ccmr2_input().modify(|_, w| w.ic4psc().bits(prescaler as u8)) }
                    };
                    Ok( () )
                }
                fn get_prescaler(&mut self, channel: Channel) -> Result<Prescaler, Self::Error> {
                    if !PINS::is_used(channel) {
                        return Err(CaptureError::UnavailableChannel);
                    }

                    Ok( Prescaler::from_register(
                        match channel {
                            Channel::C1 => unsafe { (*$TIMX::ptr()).ccmr1_input().read().ic1psc().bits() as u8 }
                            Channel::C2 => unsafe { (*$TIMX::ptr()).ccmr1_input().read().ic2psc().bits() as u8 }
                            Channel::C3 => unsafe { (*$TIMX::ptr()).ccmr2_input().read().ic3psc().bits() as u8 }
                            Channel::C4 => unsafe { (*$TIMX::ptr()).ccmr2_input().read().ic4psc().bits() as u8 }
                        }
                    ))
                }
                fn generate_event(&mut self, channel: Channel) -> Result<(), Self::Error> {
                    if !PINS::is_used(channel) {
                        return Err(CaptureError::UnavailableChannel);
                    }

                    match channel {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).egr.write(|w| w.cc1g().set_bit() ) },
                        Channel::C2 => unsafe { (*$TIMX::ptr()).egr.write(|w| w.cc2g().set_bit() ) },
                        Channel::C3 => unsafe { (*$TIMX::ptr()).egr.write(|w| w.cc3g().set_bit() ) },
                        Channel::C4 => unsafe { (*$TIMX::ptr()).egr.write(|w| w.cc4g().set_bit() ) },
                    };
                    Ok( () )
                }
                fn listen(&mut self, channel: Channel) -> Result<(), Self::Error> {
                    if !PINS::is_used(channel) {
                        return Err(CaptureError::UnavailableChannel);
                    }

                    match channel {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc1ie().set_bit() ) },
                        Channel::C2 => unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc2ie().set_bit() ) },
                        Channel::C3 => unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc3ie().set_bit() ) },
                        Channel::C4 => unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc4ie().set_bit() ) },
                    };
                    Ok( () )
                }
                fn unlisten(&mut self, channel: Channel) -> Result<(), Self::Error> {
                    if !PINS::is_used(channel) {
                        return Err(CaptureError::UnavailableChannel);
                    }

                    match channel {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc1ie().clear_bit() ) },
                        Channel::C2 => unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc2ie().clear_bit() ) },
                        Channel::C3 => unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc3ie().clear_bit() ) },
                        Channel::C4 => unsafe { (*$TIMX::ptr()).dier.modify(|_, w| w.cc4ie().clear_bit() ) },
                    };
                    Ok( () )
                }
            }
        )+
    }
}

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
hal! {
    TIM1: (tim1),
}

hal! {
    TIM2: (tim2),
    TIM3: (tim3),
}

#[cfg(feature = "medium")]
hal! {
    TIM4: (tim4),
}
