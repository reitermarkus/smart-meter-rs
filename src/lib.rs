#![deny(missing_debug_implementations)]

use std::{
  fmt,
  io::{self, Read},
  marker::PhantomData,
  num::NonZeroUsize,
};

use dlms_cosem::{
  hdlc::HdlcDataLinkLayer, mbus::MBusDataLinkLayer, Apdu, Dlms, DlmsDataLinkLayer,
  Error as DlmsError, ObisMap,
};
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
      if let Ok((_, obis)) = ObisMap::parse(&apdu) {
        return Some(Ok(obis));
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
pub struct SmartMeter<R, F> {
  reader: R,
  dlms: Dlms,
  buffer: Vec<u8>,
  _marker: PhantomData<F>,
}

impl<R, F> SmartMeter<R, F> {
  pub fn obis_iter(reader: R, dlms: Dlms) -> ObisIterator<Self> {
    Self::apdu_iter(reader, dlms).into()
  }

  pub fn apdu_iter(reader: R, dlms: Dlms) -> Self {
    Self {
      reader,
      dlms,
      buffer: Vec::new(),
      _marker: PhantomData,
    }
  }

  pub fn reader(&mut self) -> &mut R {
    &mut self.reader
  }
}

impl<R, Dll> Iterator for SmartMeter<R, Dll>
where
  R: Read,
  for<'i> Dll: SmartMeterDataLinkLayer<'i>,
{
  type Item = Result<Apdu, Error>;

  /// Get the next reading.
  fn next(&mut self) -> Option<Self::Item> {
    let mut bytes_needed = 0;
    let mut telegrams_needed = 1;

    'outer: loop {
      match (&mut self.reader)
        .take(bytes_needed as u64)
        .read_to_end(&mut self.buffer)
      {
        Ok(_) => (),
        Err(err) => return Some(Err(Error::Io(err))),
      }

      let mut telegrams = Vec::new();

      let mut buffer = self.buffer.as_slice();
      let mut telegram_1_len = 0;
      let mut telegrams_len = 0;

      for i in 0..telegrams_needed {
        let err = match Dll::parse_frame(buffer).map_err(Into::into) {
          Ok((next_buffer, telegram)) => {
            let telegram_len = buffer.len() - next_buffer.len();
            if i == 0 {
              telegram_1_len = telegram_len;
            }
            telegrams_len += telegram_len;
            buffer = next_buffer;
            telegrams.push(telegram);
            None
          }
          Err(err) => Some(err),
        };
        match err {
          Some(ParseError::Incomplete(n)) => {
            bytes_needed = n.map(NonZeroUsize::get).unwrap_or(1);
            continue 'outer;
          }
          Some(ParseError::InvalidStart) => {
            drop(telegrams);
            self.buffer.remove(0);
            bytes_needed = 0;
            continue 'outer;
          }
          Some(ParseError::Other) => {
            // Input is invalid but not incomplete,
            // so try advancing the buffer.
            drop(telegrams);
            self.buffer.remove(0);
            bytes_needed = 0;
            continue 'outer;
          }
          None => (),
        }
      }
      bytes_needed = 0;
      match Dll::decrypt(&self.dlms, &telegrams[..]) {
        Ok(apdu) => {
          drop(telegrams);
          self.buffer.drain(0..telegrams_len);
          return Some(Ok(apdu));
        }
        Err(DlmsError::Incomplete(n)) => {
          telegrams_needed += n.map(NonZeroUsize::get).unwrap_or(1);
        }
        Err(DlmsError::InvalidFormat | DlmsError::ChecksumMismatch) => {
          // Other error, continue with next telegram.
          drop(telegrams);
          self.buffer.drain(0..telegram_1_len);
          telegrams_needed = 1;
          continue;
        }
        Err(DlmsError::DecryptionFailed) => return Some(Err(Error::DecryptionFailed)),
      }
    }
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

// unfortunatly rust does not have higher rank trait bounds for types
pub trait SmartMeterDataLinkLayer<'i>: ParseFrame<'i> {
  fn decrypt(dlms: &Dlms, frames: &[<Self as ParseFrame<'i>>::Frame]) -> Result<Apdu, DlmsError>;
}

impl<'i, T> SmartMeterDataLinkLayer<'i> for T
where
  for<'f> Self: DlmsDataLinkLayer<'i, &'f [<Self as ParseFrame<'i>>::Frame]> + ParseFrame<'i>,
{
  fn decrypt(dlms: &Dlms, frames: &[<Self as ParseFrame<'i>>::Frame]) -> Result<Apdu, DlmsError> {
    dlms.decrypt_apdu::<T, _>(frames).map(|(_, apdu)| apdu)
  }
}

pub trait ParseFrame<'i> {
  type Frame;
  type Error: Into<ParseError>;
  fn parse_frame(input: &'i [u8]) -> Result<(&'i [u8], Self::Frame), Self::Error>;
}

impl<'i> ParseFrame<'i> for MBusDataLinkLayer {
  type Frame = Telegram<'i>;
  type Error = MBusError;

  fn parse_frame(input: &'i [u8]) -> Result<(&'i [u8], Self::Frame), Self::Error> {
    Telegram::parse(input)
  }
}

impl<'i> ParseFrame<'i> for HdlcDataLinkLayer {
  type Frame = HdlcFrame<'i>;
  type Error = HdlcError;

  fn parse_frame(input: &'i [u8]) -> Result<(&'i [u8], Self::Frame), Self::Error> {
    HdlcFrame::parse(input)
  }
}
