use std::error::Error;
use std::env;
use std::net::TcpStream;
use std::sync::{Arc, RwLock, Weak};
use std::thread;

use either::Either;
use hex::FromHex;
use serialport::{Parity, DataBits, StopBits};
use serde_json::json;

use dlms_cosem::{ObisCode, Data, DateTime, Dlms};
use smart_meter::SmartMeter;
use webthing::{BaseThing, BaseProperty, Thing, WebThingServer, Action, ThingsType, server::ActionGenerator};

struct Generator;

impl ActionGenerator for Generator {
  fn generate(&self,
      _thing: Weak<RwLock<Box<dyn Thing>>>,
      _name: String,
      _input: Option<&serde_json::Value>,
  ) -> Option<Box<dyn Action>> {
    None
  }
}

#[actix_rt::main]
async fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
  let url_or_path = env::var("SERIAL_PORT").unwrap_or("/dev/serial0");
  let key = env::var("KEY").expect("No key provided");
  let key = <[u8; 16]>::from_hex(key).expect("Invalid key format");
  let port = env::var("PORT").map(|s| s.parse::<u16>().expect("Port is invalid")).unwrap_or(8888);

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

  let smart_meter = SmartMeter::new(stream, dlms);

  let mut smart_meter = smart_meter.map(|res| match res {
    Ok(mut obis) => {
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

      Ok(obis)
    },
    err => err,
  });

  let mut thing = BaseThing::new(
      "urn:dev:ops:smart-meter-1".to_owned(),
      "Smart Meter".to_owned(),
      Some(vec!["MultiLevelSensor".to_owned()]),
      Some("A smart energy meter".to_owned()),
  );

  let first_response = smart_meter.next().unwrap()?;

  for (obis_code, reg) in first_response.iter() {
    let level_description = json!({
        "@type": "LevelProperty",
        "title": obis_code,
        "type": "number",
        "unit": reg.unit(),
        "readOnly": true
    });
    let level_description = level_description.as_object().unwrap().clone();
    thing.add_property(Box::new(BaseProperty::new(
        obis_code.to_string(),
        serde_json::to_value(reg.value())?,
        None,
        Some(level_description),
    )));
  }

  let thing: Arc<RwLock<Box<dyn Thing>>> = Arc::new(RwLock::new(Box::new(thing)));
  let thing_clone = thing.clone();

  thread::spawn(move || {
    let thing = thing_clone;

    while let Some(res) = smart_meter.next() {
      let obis = res.unwrap();

      let mut thing = thing.write().unwrap();
      for (obis_code, reg) in obis.iter() {
        let property_name = obis_code.to_string();
        let new_value = serde_json::to_value(reg.value()).unwrap();
        let prop = thing.find_property(&property_name).unwrap();
        let _ = prop.set_cached_value(new_value.clone());
        thing.property_notify(property_name, new_value);
      }
    }
  });

  let mut server = WebThingServer::new(
      ThingsType::Single(thing),
      Some(port),
      None,
      None,
      Box::new(Generator),
      None,
      Some(true),
  );
  Ok(server.start(None).await?)
}
