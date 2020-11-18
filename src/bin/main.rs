#![no_main]
#![no_std]

use breathe_rs as _; // global logger + panicking-behavior + memory layout
use cortex_m::peripheral::DWT;
use rtic::cyccnt::U32Ext as _;

use stm32f4xx_hal as hal;

use breathe_rs::scd30::SCD30;
use core::convert::TryInto;
use ds1307::{Datelike, Ds1307, NaiveDate, Rtcc, Timelike};
use stm32f4xx_hal::gpio::gpiob::{PB, PB8, PB9};
use stm32f4xx_hal::gpio::{AlternateOD, Output, PushPull, AF4};
use stm32f4xx_hal::hal::blocking::i2c::{Read, Write, WriteRead};
use stm32f4xx_hal::i2c::I2c;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::stm32::I2C1;

const PERIOD: u32 = 84_000_000;

type RTC = Ds1307<I2c<I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led: PB<Output<PushPull>>,
    }

    #[init(schedule = [blink])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core: rtic::Peripherals = cx.core;
        let device: stm32::Peripherals = cx.device;

        // enable CYCCNT
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let rcc = device.RCC.constrain().cfgr.sysclk(84.mhz()).hclk(84.mhz());
        let clocks = rcc.freeze();

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();

        let mut led = gpiob.pb12.into_push_pull_output().downgrade();

        led.set_low().unwrap();

        let now = cx.start;
        cx.schedule.blink(now + PERIOD.cycles()).unwrap();

        let sda = gpiob.pb9.into_alternate_af4_open_drain();
        let scl = gpiob.pb8.into_alternate_af4_open_drain();

        let i2c = hal::i2c::I2c::i2c1(device.I2C1, (scl, sda), 100.khz(), clocks);
        // let mut rtc = Ds1307::new(i2c);
        // // let datetime = NaiveDate::from_ymd(2020, 5, 2).and_hms(19, 59, 58);
        // // defmt::error!("a");
        // // rtc.set_datetime(&datetime).unwrap();

        let mut sensor = SCD30::init(i2c);
        let v = sensor.read_fw_version().unwrap();
        defmt::error!("maj: {:u8} min: {:u8}", v[0], v[1]);
        sensor
            .start_continuous_measurement(PRESSURE_OFFSET)
            .unwrap();

        loop {
            defmt::info!("waiting for data ready.");
            loop {
                if sensor.get_data_ready().unwrap() {
                    break;
                }
            }
            defmt::info!("data is ready.");

            let data = sensor.read_measurement().unwrap();
            defmt::info!(
                "
        CO2  {:f32} PPM
        Temp {:f32} C
        Hum  {:f32} % 
        ",
                data.co2,
                data.temperature,
                data.humidity
            );
        }

        init::LateResources { led }
    }

    #[idle(resources = [])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(resources = [led], schedule = [blink])]
    fn blink(cx: blink::Context) {
        let led: &mut PB<Output<PushPull>> = cx.resources.led;
        if led.is_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }

        cx.schedule.blink(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
