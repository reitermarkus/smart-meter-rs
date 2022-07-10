#![deny(missing_debug_implementations)]

use std::fmt;
use std::io::{self, Read};
use std::num::NonZeroUsize;

use dlms_cosem::hdlc::HdlcDataLinkLayer;
use dlms_cosem::{mbus::MBusDataLinkLayer, Apdu, Dlms, Error as DlmsError, ObisMap};
use hdlcparse::{type3::HdlcFrame, Error as HdlcError};
use mbusparse::{Error as MBusError, Telegram};

#[derive(Debug)]
pub enum Error {
  Io(io::Error),
  DecryptionFailed,
}

impl fmt::Display for Error {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::Io(err) => err.fmt(f),
      Self::DecryptionFailed => write!(f, "decryption failed"),
    }
  }
}

impl std::error::Error for Error {}

#[derive(Debug)]
pub struct ObisIterator<I> {
  iter: I,
}

impl<I> Iterator for ObisIterator<I>
where
  I: Iterator<Item = Result<Apdu, Error>>,
{
  type Item = Result<ObisMap, Error>;

  fn next(&mut self) -> Option<Self::Item> {
    for apdu in &mut self.iter {
      let apdu = match apdu {
        Ok(apdu) => apdu,
        Err(err) => return Some(Err(err)),
      };
      match ObisMap::parse(&apdu) {
        Ok((_, obis)) => return Some(Ok(obis)),
        Err(_) => (),
      }
    }
    None
  }
}

impl<I> From<I> for ObisIterator<I> {
  fn from(iter: I) -> Self {
    Self { iter }
  }
}

#[derive(Debug)]
pub struct MBusSmartMeter<R> {
  reader: R,
  data_link_layer: MBusSmartMeterDataLinkLayer,
  buffer: Vec<u8>,
}

impl<R> MBusSmartMeter<R> {
  pub fn new(reader: R, dlms: Dlms<MBusDataLinkLayer>) -> ObisIterator<Self> {
    Self {
      reader,
      data_link_layer: MBusSmartMeterDataLinkLayer { dlms },
      buffer: Vec::new(),
    }
    .into()
  }

  pub fn new_apdu(reader: R, dlms: Dlms<MBusDataLinkLayer>) -> Self {
    Self {
      reader,
      data_link_layer: MBusSmartMeterDataLinkLayer { dlms },
      buffer: Vec::new(),
    }
  }
  pub fn reader(&mut self) -> &mut R {
    &mut self.reader
  }
}

impl<R> Iterator for MBusSmartMeter<R>
where
  R: Read,
{
  type Item = Result<Apdu, Error>;

  /// Get the next reading.
  fn next(&mut self) -> Option<Self::Item> {
    parse_next(&mut self.buffer, &mut self.reader, &self.data_link_layer)
  }
}

#[derive(Debug)]
pub enum ParseError {
  Incomplete(Option<NonZeroUsize>),
  InvalidStart,
  Other,
}

impl From<MBusError> for ParseError {
  fn from(err: MBusError) -> Self {
    use MBusError::*;
    match err {
      InvalidStartCharacter => ParseError::InvalidStart,
      InvalidFormat | ChecksumMismatch => ParseError::Other,
      Incomplete(n) => ParseError::Incomplete(n),
    }
  }
}

impl From<HdlcError> for ParseError {
  fn from(err: HdlcError) -> Self {
    use HdlcError::*;
    match err {
      InvalidStartCharacter => ParseError::InvalidStart,
      InvalidFormat | InvalidAddress | InvalidChecksum => ParseError::Other,
      Incomplete(n) => ParseError::Incomplete(n),
    }
  }
}

pub trait SmartMeterDataLinkLayer<'i> {
  type Frame: 'i;
  type Error: Into<ParseError>;
  fn parse_frame(input: &'i [u8]) -> Result<(&'i [u8], Self::Frame), Self::Error>;
  fn decrypt(&self, frames: &[Self::Frame]) -> Result<Apdu, DlmsError>;
}

#[derive(Debug)]
struct MBusSmartMeterDataLinkLayer {
  dlms: Dlms<MBusDataLinkLayer>,
}

impl<'i> SmartMeterDataLinkLayer<'i> for MBusSmartMeterDataLinkLayer {
  type Frame = Telegram<'i>;
  type Error = MBusError;

  fn parse_frame(input: &'i [u8]) -> Result<(&'i [u8], Self::Frame), Self::Error> {
    Telegram::parse(input)
  }

  fn decrypt(&self, frames: &[Self::Frame]) -> Result<Apdu, DlmsError> {
    self.dlms.decrypt_apdu(frames).map(|(_, apdu)| apdu)
  }
}

#[derive(Debug)]
struct HdlcSmartMeterDataLinkLayer {
  dlms: Dlms<HdlcDataLinkLayer>,
}

impl<'i> SmartMeterDataLinkLayer<'i> for HdlcSmartMeterDataLinkLayer {
  type Frame = HdlcFrame<'i>;
  type Error = HdlcError;
  fn parse_frame(input: &'i [u8]) -> Result<(&'i [u8], HdlcFrame<'i>), HdlcError> {
    HdlcFrame::parse(input)
  }
  fn decrypt(&self, frames: &[Self::Frame]) -> Result<Apdu, DlmsError> {
    self.dlms.decrypt_apdu(frames).map(|(_, apdu)| apdu)
  }
}

fn parse_next<R, Smdll>(
  buffer: &mut Vec<u8>,
  reader: &mut R,
  parse_frame: &Smdll,
) -> Option<Result<Apdu, Error>>
where
  R: Read,
  Smdll: for<'i> SmartMeterDataLinkLayer<'i>,
{
  let mut bytes_needed = 0;
  let mut telegrams_needed = 1;

  'outer: loop {
    match (reader).take(bytes_needed as u64).read_to_end(buffer) {
      Ok(_) => (),
      Err(err) => return Some(Err(Error::Io(err))),
    }

    let mut telegrams = Vec::new();

    let mut buf = buffer.as_slice();
    let mut telegram_1_len = 0;
    let mut telegrams_len = 0;

    for i in 0..telegrams_needed {
      let err = match Smdll::parse_frame(buf).map_err(|err| err.into()) {
        Ok((next_buffer, telegram)) => {
          let telegram_len = buf.len() - next_buffer.len();
          if i == 0 {
            telegram_1_len = telegram_len;
          }
          telegrams_len += telegram_len;
          buf = next_buffer;
          telegrams.push(telegram);
          None
        }
        Err(err) => Some(err),
      };
      match err {
        Some(ParseError::Incomplete(n)) => {
          bytes_needed = n.map(|b| b.get()).unwrap_or(1);
          continue 'outer;
        }
        Some(ParseError::InvalidStart) => {
          drop(telegrams);
          buffer.remove(0);
          bytes_needed = 0;
          continue 'outer;
        }
        Some(ParseError::Other) => {
          // Input is invalid but not incomplete,
          // so try advancing the buffer.
          drop(telegrams);
          buffer.remove(0);
          bytes_needed = 0;
          continue 'outer;
        }
        None => (),
      }
    }
    bytes_needed = 0;

    match parse_frame.decrypt(&telegrams) {
      Ok(apdu) => {
        drop(telegrams);
        buffer.drain(0..telegrams_len);
        return Some(Ok(apdu));
      }
      Err(DlmsError::Incomplete(n)) => {
        telegrams_needed += n.map(|t| t.get()).unwrap_or(1);
      }
      Err(DlmsError::InvalidFormat | DlmsError::ChecksumMismatch) => {
        // Other error, continue with next telegram.
        drop(telegrams);
        buffer.drain(0..telegram_1_len);
        telegrams_needed = 1;
        continue;
      }
      Err(DlmsError::DecryptionFailed) => return Some(Err(Error::DecryptionFailed)),
    }
  }
}

#[derive(Debug)]
pub struct HdlcSmartMeter<R> {
  reader: R,
  data_link_layer: HdlcSmartMeterDataLinkLayer,
  buffer: Vec<u8>,
}

impl<R> HdlcSmartMeter<R> {
  pub fn new(reader: R, dlms: Dlms<HdlcDataLinkLayer>) -> ObisIterator<Self> {
    Self {
      reader,
      data_link_layer: HdlcSmartMeterDataLinkLayer { dlms },
      buffer: Vec::new(),
    }
    .into()
  }

  pub fn apdu_iter(reader: R, dlms: Dlms<HdlcDataLinkLayer>) -> Self {
    Self {
      reader,
      data_link_layer: HdlcSmartMeterDataLinkLayer { dlms },
      buffer: Vec::new(),
    }
  }
  pub fn reader(&mut self) -> &mut R {
    &mut self.reader
  }
}

impl<R> Iterator for HdlcSmartMeter<R>
where
  R: Read,
{
  type Item = Result<Apdu, Error>;

  fn next(&mut self) -> Option<Self::Item> {
    parse_next(&mut self.buffer, &mut self.reader, &self.data_link_layer)
  }
}
