use byteorder::{ByteOrder, LittleEndian};
use ds323x::{NaiveDate, NaiveDateTime, NaiveTime};

const DATE_SIZE: usize = 7; // 1 - hour, 1 - minute, 1 - second, 1 - day, 1 - month, 2 - year
pub struct USBDateBuffer {
    buffer: [u8; DATE_SIZE],
    position: usize,
    start_byte_found: bool,
}

impl USBDateBuffer {
    pub fn new() -> Self {
        USBDateBuffer {
            buffer: [0; DATE_SIZE],
            position: 0,
            start_byte_found: false,
        }
    }

    pub fn recv(&mut self, data: &[u8]) {
        for byte in data.iter() {
            if !self.start_byte_found && *byte == 0xaa {
                self.start_byte_found = true;
            } else if self.start_byte_found {
                if self.position < DATE_SIZE {
                    self.buffer[self.position] = *byte;
                    self.position += 1;
                }
            }
        }
    }

    pub fn is_date_available(&self) -> bool {
        self.position == DATE_SIZE
    }

    pub fn clear(&mut self) {
        self.position = 0;
    }

    pub fn get_datetime(&self) -> NaiveDateTime {
        NaiveDateTime::new(
            NaiveDate::from_ymd(
                LittleEndian::read_u16(&self.buffer[5..]) as i32,
                self.buffer[4] as u32,
                self.buffer[3] as u32,
            ),
            NaiveTime::from_hms(
                self.buffer[0] as u32,
                self.buffer[1] as u32,
                self.buffer[2] as u32,
            ),
        )
    }
}
