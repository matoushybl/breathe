use crate::scd30::SensorData;
use arrayvec::ArrayString;
use core::fmt::Debug;
use core::fmt::Write;
use core::ops::Add;
use ds323x::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike};
use embedded_graphics::fonts::Font12x16;
use embedded_graphics::image::Image;
use embedded_graphics::{
    fonts::{Font24x32, Text},
    prelude::*,
    text_style,
};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use epd_waveshare::color::*;
use epd_waveshare::epd4in2::{Display4in2, EPD4in2};
use epd_waveshare::prelude::*;
use tinybmp::Bmp;

const WIDTH: i32 = 400;
const HEIGHT: i32 = 300;

const SENSOR_DATA_LEN: usize = 10;
#[derive(Copy, Clone)]
pub struct State {
    sensor_data: [SensorData; SENSOR_DATA_LEN],
    sensor_data_index: usize,
    pub datetime: NaiveDateTime,
}

impl Default for State {
    fn default() -> Self {
        State {
            sensor_data: Default::default(),
            sensor_data_index: 0,
            datetime: NaiveDateTime::new(
                NaiveDate::from_ymd(2020, 1, 1),
                NaiveTime::from_hms(10, 10, 10),
            ),
        }
    }
}

impl State {
    pub fn update_sensor_data(&mut self, data: SensorData) {
        self.sensor_data[self.sensor_data_index] = data;
        self.sensor_data_index += 1;
        if self.sensor_data_index == SENSOR_DATA_LEN {
            self.sensor_data_index = 0;
        }
    }

    pub fn get_averaged_sensor_data(&self) -> (f32, f32, f32) {
        let (co2, temp, humi) =
            self.sensor_data
                .iter()
                .fold((0.0f32, 0.0f32, 0.0f32), |acc, data| {
                    (
                        acc.0 + data.co2,
                        acc.1 + data.temperature,
                        acc.2 + data.humidity,
                    )
                });
        (
            co2 / SENSOR_DATA_LEN as f32,
            temp / SENSOR_DATA_LEN as f32,
            humi / SENSOR_DATA_LEN as f32,
        )
    }
}

pub struct Renderer<SPI, CS, BUSY, DC, RST> {
    driver: EPD4in2<SPI, CS, BUSY, DC, RST>,
    pub display: Display4in2,
}

impl<SPI, CS, BUSY, DC, RST, E> Renderer<SPI, CS, BUSY, DC, RST>
where
    E: Debug,
    SPI: embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: OutputPin,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    pub fn new(
        spi: &mut SPI,
        driver: EPD4in2<SPI, CS, BUSY, DC, RST>,
        display: Display4in2,
    ) -> Self {
        let mut s = Self { driver, display };
        s.display.set_rotation(DisplayRotation::Rotate0);
        s.driver.set_lut(spi, Some(RefreshLUT::FULL)).unwrap();
        s.driver
            .update_and_display_frame(spi, &s.display.buffer())
            .unwrap();
        s
    }

    pub fn render_boot(&mut self, spi: &mut SPI) {
        self.display.clear_buffer(Color::White);
        Self::draw_text(&mut self.display, "Booting", WIDTH / 2 - 70, HEIGHT / 2);
        self.driver
            .update_and_display_frame(spi, self.display.buffer())
            .unwrap();
    }

    pub fn render_data(&mut self, spi: &mut SPI, state: &State) {
        let mut buf = ArrayString::<[_; 30]>::new();
        self.display.clear_buffer(Color::White);
        Self::draw_text(&mut self.display, "Breathe...", 10, 10);

        buf.clear();
        let averaged_values = state.get_averaged_sensor_data();
        write!(&mut buf, "CO2:  {:.1} PPM", averaged_values.0).unwrap();
        Self::draw_text(&mut self.display, &buf, 10, 80);

        buf.clear();
        write!(&mut buf, "Temp: {:.1} C", averaged_values.1).unwrap();
        Self::draw_text(&mut self.display, &buf, 10, 110);

        buf.clear();
        write!(&mut buf, "Humi: {:.1} %", averaged_values.2).unwrap();
        Self::draw_text(&mut self.display, &buf, 10, 140);

        let bmp = Bmp::from_slice(include_bytes!("res/rust-social.bmp"))
            .expect("Failed to parse BMP image");
        let image = Image::new(&bmp, Point::new(WIDTH - 50, HEIGHT - 50));
        image.draw(&mut self.display);

        let bmp = if averaged_values.0 < 1000.0 {
            Bmp::from_slice(include_bytes!("res/happy.bmp")).expect("Failed to parse BMP image")
        } else if averaged_values.0 < 1500.0 {
            Bmp::from_slice(include_bytes!("res/neutral.bmp")).expect("Failed to parse BMP image")
        } else {
            Bmp::from_slice(include_bytes!("res/sad.bmp")).expect("Failed to parse BMP image")
        };
        let image = Image::new(&bmp, Point::new(WIDTH / 2 - 50, HEIGHT - 110));
        image.draw(&mut self.display).unwrap();

        buf.clear();
        write!(
            &mut buf,
            "{}.{}.{}",
            state.datetime.date().day(),
            state.datetime.date().month(),
            state.datetime.date().year()
        )
        .unwrap();

        let _ = Text::new(&mut buf, Point::new(10, HEIGHT - 25))
            .into_styled(text_style!(
                font = Font12x16,
                text_color = Black,
                background_color = White
            ))
            .draw(&mut self.display);

        buf.clear();
        write!(
            &mut buf,
            "{:2}:{:02}",
            state.datetime.time().hour(),
            state.datetime.time().minute(),
        )
        .unwrap();

        let _ = Text::new(&mut buf, Point::new(WIDTH - 120, 10))
            .into_styled(text_style!(
                font = Font24x32,
                text_color = Black,
                background_color = White
            ))
            .draw(&mut self.display);

        self.driver
            .update_and_display_frame(spi, self.display.buffer())
            .unwrap();
    }

    fn draw_text(display: &mut Display4in2, text: &str, x: i32, y: i32) {
        let _ = Text::new(text, Point::new(x, y))
            .into_styled(text_style!(
                font = Font24x32,
                text_color = Black,
                background_color = White
            ))
            .draw(display);
    }
}
