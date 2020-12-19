use std::time::{Duration, Instant};
use serialport::FlowControl;
use chrono::{DateTime, NaiveDateTime, Utc, Timelike, Local, Datelike};
use byteorder::{LittleEndian, ByteOrder};
use std::io::Write;

fn main() {
    println!("Hello, world!");
    let mut settings = serialport::SerialPortSettings::default();
    settings.baud_rate = 115200;
    settings.timeout = Duration::from_millis(10);
    settings.flow_control = FlowControl::None;
    let mut port = serialport::open_with_settings("/dev/ttyACM0", &settings).unwrap();

    let date = Local::now().naive_local();
    let mut buffer = [0u8; 8];
    // 1 - hour, 1 - minute, 1 - second, 1 - day, 1 - month, 2 - year
    buffer[0] = 0xaa;
    buffer[1] = date.hour() as u8;
    buffer[2] = date.minute() as u8;
    buffer[3] = date.second() as u8;
    buffer[4] = date.day() as u8;
    buffer[5] = date.month() as u8;
    LittleEndian::write_u16(&mut buffer[6..], date.year() as u16);
    println!("date: {:?}", buffer);

    port.write(&buffer).unwrap();
}
