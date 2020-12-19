#![no_main]
#![no_std]

use breathe_rs as _; // global logger + panicking-behavior + memory layout
use cortex_m::peripheral::DWT;
use rtic::cyccnt::U32Ext as _;

use stm32f4xx_hal as hal;

use breathe_rs::scd30::{SensorData, SCD30};
// use ds1307::{Datelike, Ds1307, NaiveDate, Rtcc, Timelike};
use stm32f4xx_hal::gpio::gpiob::{PB, PB13, PB15, PB4, PB8, PB9};
use stm32f4xx_hal::gpio::{
    Alternate, AlternateOD, Floating, Input, Output, PushPull, AF4, AF5, AF9,
};
use stm32f4xx_hal::i2c::I2c;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::stm32::{I2C1, I2C3};

use breathe_rs::rendering::{Renderer, State};
use breathe_rs::usb_parser::USBDateBuffer;
use ds323x::ic::DS3231;
use ds323x::interface::I2cInterface;
use ds323x::{Datelike, Ds323x, NaiveDate, NaiveDateTime, Rtcc, Timelike};
use embedded_hal::blocking::delay::DelayMs;
use epd_waveshare::epd4in2::{Display4in2, EPD4in2};
use epd_waveshare::prelude::WaveshareDisplay;
use stm32f4xx_hal::gpio::gpioa::{PA, PA8};
use stm32f4xx_hal::otg_fs::*;
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::spi::{NoMiso, Spi};
use stm32f4xx_hal::timer::Timer;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

const SECOND: u32 = 84_000_000;

const DISPLAY_REFRESH: u32 = SECOND;
const BLINK_PERIOD: u32 = SECOND / 10;
const UPDATE_SENSOR: u32 = 2 * SECOND;

const DISPLAY_REFRESH_COUNT: u16 = 20;

const PRESSURE_OFFSET: u16 = 1031; // Brno, 17.11

struct Delay {
    timer: Timer<stm32::TIM10>,
}

impl Delay {
    pub fn new(tim: stm32::TIM10, clocks: Clocks) -> Self {
        let timer = Timer::tim10(tim, 1000.hz(), clocks);
        Self { timer }
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        let _ = self.timer.wait(); // clear UIF flag and reset counting
        for _ in 0..ms {
            nb::block!(self.timer.wait());
        }
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        let _ = self.timer.wait(); // clear UIF flag and reset counting
        for _ in 0..ms {
            nb::block!(self.timer.wait());
        }
    }
}

pub type EPDisplay = Renderer<
    SPI,
    PB<Output<PushPull>>,
    PB<Input<Floating>>,
    PB<Output<PushPull>>,
    PB<Output<PushPull>>,
>;
type SPI = Spi<stm32::SPI2, (PB13<Alternate<AF5>>, NoMiso, PB15<Alternate<AF5>>)>;

type CO2Sensor = SCD30<I2c<I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>>;
type RTC = Ds323x<I2cInterface<I2c<I2C3, (PA8<AlternateOD<AF4>>, PB4<AlternateOD<AF9>>)>>, DS3231>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led: PA<Output<PushPull>>,
        sensor: CO2Sensor,
        display: EPDisplay,
        spi: SPI,
        rtc: RTC,
        state: State,
        #[init(0)]
        second_counter: u16,
        serial: SerialPort<'static, UsbBusType>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        date_buffer: USBDateBuffer,
    }

    #[init(schedule = [blink, update_sensor, refresh_display])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

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
        let _gpioc = device.GPIOC.split();

        let mut led = gpioa.pa7.into_push_pull_output().downgrade();

        let sda = gpiob.pb9.into_alternate_af4_open_drain();
        let scl = gpiob.pb8.into_alternate_af4_open_drain();
        let cs = gpiob.pb10.into_push_pull_output().downgrade();
        let spiclk = gpiob.pb13.into_alternate_af5();
        let spimosi = gpiob.pb15.into_alternate_af5();
        let dc = gpiob.pb2.into_push_pull_output().downgrade();
        let busy = gpiob.pb0.into_floating_input().downgrade();
        let rst = gpiob.pb1.into_push_pull_output().downgrade();

        let sda3 = gpiob.pb4.into_alternate_af9_open_drain();
        let scl3 = gpioa.pa8.into_alternate_af4_open_drain();

        defmt::error!("startup");

        let i2c3 = I2c::i2c3(device.I2C3, (scl3, sda3), 100.khz(), clocks);
        let mut rtc = ds323x::Ds323x::new_ds3231(i2c3);

        let initial_datetime = rtc.get_datetime().unwrap();
        defmt::error!("startup");

        let mut spi = hal::spi::Spi::spi2(
            device.SPI2,
            (spiclk, NoMiso, spimosi),
            embedded_hal::spi::MODE_0,
            4.mhz().into(),
            clocks,
        );

        let mut delay = Delay::new(device.TIM10, clocks);

        let epd4in2 =
            EPD4in2::new(&mut spi, cs, busy, dc, rst, &mut delay).expect("eink initalize error");
        let display = Display4in2::default();
        let mut display = Renderer::new(&mut spi, epd4in2, display);
        display.render_boot(&mut spi);

        // the i2c must be initialized after all of the pins are initialized, otherwise it won't boot after startup.
        // There might be another reason, but the solution is to reorder initialization of peripherals.
        let i2c = hal::i2c::I2c::i2c1(device.I2C1, (scl, sda), 45.khz(), clocks);
        let mut sensor = SCD30::init(i2c);

        delay.delay_ms(2000u16);

        if let Ok(v) = sensor.read_fw_version() {
            defmt::error!("maj: {:u8} min: {:u8}", v[0], v[1]);
        }

        sensor
            .start_continuous_measurement(PRESSURE_OFFSET)
            .unwrap();

        let usb = USB {
            usb_global: device.OTG_FS_GLOBAL,
            usb_device: device.OTG_FS_DEVICE,
            usb_pwrclk: device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
        };

        *USB_BUS = Some(UsbBus::new(usb, &mut EP_MEMORY[..]));

        let serial = usbd_serial::SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        let now = cx.start;
        cx.schedule.blink(now + BLINK_PERIOD.cycles()).unwrap();
        cx.schedule
            .update_sensor(now + UPDATE_SENSOR.cycles())
            .unwrap();
        cx.schedule
            .refresh_display(now + DISPLAY_REFRESH.cycles())
            .unwrap();

        init::LateResources {
            led,
            sensor,
            display,
            spi,
            rtc,
            state: State::default(),
            serial,
            usb_dev,
            date_buffer: USBDateBuffer::new(),
        }
    }

    #[idle(resources = [date_buffer, rtc])]
    fn idle(cx: idle::Context) -> ! {
        let mut date_buffer = cx.resources.date_buffer;
        let mut rtc = cx.resources.rtc;
        loop {
            date_buffer.lock(|buf: &mut USBDateBuffer| {
                if buf.is_date_available() {
                    let date = buf.get_datetime();
                    rtc.lock(|rtc: &mut RTC| {
                        rtc.set_datetime(&date).unwrap();
                    });
                    buf.clear();
                }
            });
        }
    }

    #[task(binds = OTG_FS, resources = [serial, usb_dev, date_buffer])]
    fn usb_handler(cx: usb_handler::Context) {
        let serial: &mut SerialPort<UsbBusType> = cx.resources.serial;
        let usb_dev: &mut UsbDevice<UsbBusType> = cx.resources.usb_dev;
        let date_buffer: &mut USBDateBuffer = cx.resources.date_buffer;
        if !usb_dev.poll(&mut [serial]) {
            return;
        }
        let mut buf = [0u8; 64];

        match serial.read(&mut buf[..]) {
            Ok(count) => {
                date_buffer.recv(&buf[..count]);
                // for c in buf[..count].iter() {
                //     serial.write(&[*c]).unwrap();
                //     defmt::warn!("data {:u8}", c);
                // }
                // count bytes were read to &buf[..count]
            }
            Err(UsbError::WouldBlock) => {
                // defmt::debug!("would block.");
            } // No data received
            Err(_) => {
                defmt::warn!("err.");
            } // An error occurred
        };
    }

    #[task(resources = [led], schedule = [blink])]
    fn blink(cx: blink::Context) {
        let led: &mut PA<Output<PushPull>> = cx.resources.led;
        if led.is_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }

        cx.schedule
            .blink(cx.scheduled + BLINK_PERIOD.cycles())
            .unwrap();
    }

    #[task(resources = [sensor, state, rtc], schedule = [update_sensor])]
    fn update_sensor(cx: update_sensor::Context) {
        let sensor: &mut CO2Sensor = cx.resources.sensor;
        let state: &mut State = cx.resources.state;
        let rtc: &mut RTC = cx.resources.rtc;

        cx.schedule
            .update_sensor(cx.scheduled + UPDATE_SENSOR.cycles())
            .unwrap();
        if !sensor.get_data_ready().unwrap_or(false) {
            return;
        }

        if let Ok(data) = sensor.read_measurement() {
            state.update_sensor_data(data);
        }

        state.datetime = rtc.get_datetime().unwrap();
    }

    #[task(resources = [display, spi, state, second_counter], schedule = [refresh_display])]
    fn refresh_display(cx: refresh_display::Context) {
        let display: &mut EPDisplay = cx.resources.display;
        let spi: &mut SPI = cx.resources.spi;
        let state: &mut State = cx.resources.state;
        let second_counter: &mut u16 = cx.resources.second_counter;

        if *second_counter == DISPLAY_REFRESH_COUNT {
            display.render_data(spi, state);
            *second_counter = 0;
        } else {
            *second_counter += 1;
        }

        cx.schedule
            .refresh_display(cx.scheduled + DISPLAY_REFRESH.cycles())
            .unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
