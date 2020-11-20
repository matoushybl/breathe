use crate::scd30::SensorData;
use arrayvec::ArrayString;
use core::fmt::Debug;
use core::fmt::Write;
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

    pub fn render_data(&mut self, spi: &mut SPI, sensor_data: &SensorData) {
        let mut buf = ArrayString::<[_; 30]>::new();
        self.display.clear_buffer(Color::White);
        Self::draw_text(&mut self.display, "Breathe...", 10, 10);

        buf.clear();
        write!(&mut buf, "CO2:  {:.1} PPM", sensor_data.co2).unwrap();
        Self::draw_text(&mut self.display, &buf, 10, 80);

        buf.clear();
        write!(&mut buf, "Temp: {:.1} C", sensor_data.temperature).unwrap();
        Self::draw_text(&mut self.display, &buf, 10, 110);

        buf.clear();
        write!(&mut buf, "Humi: {:.1} %", sensor_data.humidity).unwrap();
        Self::draw_text(&mut self.display, &buf, 10, 140);

        let bmp =
            Bmp::from_slice(include_bytes!("rust-social.bmp")).expect("Failed to parse BMP image");
        let image = Image::new(&bmp, Point::new(WIDTH - 50, HEIGHT - 50));
        image.draw(&mut self.display);

        let bmp = if sensor_data.co2 < 1000.0 {
            Bmp::from_slice(include_bytes!("happy.bmp")).expect("Failed to parse BMP image")
        } else if sensor_data.co2 < 1500.0 {
            Bmp::from_slice(include_bytes!("neutral.bmp")).expect("Failed to parse BMP image")
        } else {
            Bmp::from_slice(include_bytes!("sad.bmp")).expect("Failed to parse BMP image")
        };
        let image = Image::new(&bmp, Point::new(WIDTH / 2 - 50, HEIGHT - 110));
        image.draw(&mut self.display).unwrap();

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
