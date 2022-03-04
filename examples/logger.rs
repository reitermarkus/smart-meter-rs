use std::error::Error;
use std::env;
use std::net::TcpStream;

use either::Either;
use hex::FromHex;
use serialport::{Parity, DataBits, StopBits};

use dlms_cosem::{ObisCode, Data, DateTime, Dlms, Unit};
use smart_meter::SmartMeter;

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
  let url_or_path = env::args().nth(1).unwrap_or("/dev/serial0".into());
  let key = env::args().nth(2).expect("No key provided");
  let key = <[u8; 16]>::from_hex(key).expect("Invalid key format");

  let stream = if url_or_path.contains(":") {
    Either::Left(TcpStream::connect(url_or_path)?)
  } else {
    Either::Right(serialport::new(url_or_path, 2400)
      .parity(Parity::Even)
      .data_bits(DataBits::Eight)
      .stop_bits(StopBits::One)
      .open()?)
  };

  let dlms = Dlms::new(key);

  let mut smart_meter = SmartMeter::new(stream, dlms);

  loop {
    let mut obis = smart_meter.next().unwrap()?;

    let convert_date_time = |value| match value {
      Data::OctetString(value) => Data::DateTime(DateTime::parse(&value).unwrap().1),
      value => value,
    };
    obis.convert(&ObisCode::new(0, 0, 1, 0, 0, 255), convert_date_time);

    let convert_string = |value| match value {
      Data::OctetString(value) => Data::Utf8String(String::from_utf8(value).unwrap()),
      value => value,
    };
    obis.convert(&ObisCode::new(0, 0, 42, 0, 0, 255), convert_string);
    obis.convert(&ObisCode::new(0, 0, 96, 1, 0, 255), convert_string);

    for (key, reg) in obis.iter() {
      print!("{:<16} ", format!("{}:", key));

      fn format_value(value: &Data, unit: Option<&Unit>) -> String {
        let value = match value {
          Data::Utf8String(s) => format!("{}", s),
          Data::DateTime(date_time) => format!("{}", date_time),
          Data::Float32(n) => format!("{}", n),
          Data::Float64(n) => format!("{}", n),
          Data::LongUnsigned(n) => format!("{}", n),
          Data::DoubleLongUnsigned(n) => format!("{}", n),
          data => format!("{:?}", data),
        };

        if let Some(unit) = unit.and_then(|u| u.as_str()) {
          format!("{} {}", value, unit)
        } else {
          value
        }
      }

      println!("{}", format_value(reg.value(), reg.unit()));
    }

    println!();
  }
}
