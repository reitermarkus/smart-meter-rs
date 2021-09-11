#![deny(missing_debug_implementations)]

use std::io::{self, Read};
use std::fmt;

use dlms_cosem::{Error as DlmsError, Dlms, ObisMap};
use mbusparse::{Error as MbusError, Telegram};

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
pub struct SmartMeter<R> {
  reader: R,
  dlms: Dlms,
  buffer: Vec<u8>,
}

impl<R> SmartMeter<R> {
  pub fn new(reader: R, dlms: Dlms) -> Self {
    SmartMeter { reader, dlms, buffer: Vec::new() }
  }
}

impl<R: Read> Iterator for SmartMeter<R> {
  type Item = Result<ObisMap, Error>;

  /// Get the next reading.
  fn next(&mut self) -> Option<Self::Item> {
    let mut bytes_needed = 0;
    let mut telegrams_needed = 1;

    'outer: loop {
      match (&mut self.reader).take(bytes_needed as u64).read_to_end(&mut self.buffer) {
        Ok(_) => (),
        Err(err) => return Some(Err(Error::Io(err))),
      }

      let mut telegrams = Vec::new();

      let mut buffer = self.buffer.as_slice();
      let mut telegram_1_len = 0;
      let mut telegrams_len = 0;

      for i in 0..telegrams_needed {
        match Telegram::parse(buffer) {
          Ok((next_buffer, telegram)) => {
            let telegram_len = buffer.len() - next_buffer.len();
            if i == 0 {
              telegram_1_len = telegram_len;
            }
            telegrams_len += telegram_len;
            buffer = next_buffer;
            telegrams.push(telegram);
          },
          Err(MbusError::Incomplete(n)) => {
            bytes_needed = n.map(|b| b.get()).unwrap_or(1);
            continue 'outer
          },
          Err(MbusError::InvalidStartCharacter) => {
            self.buffer.remove(0);
            bytes_needed = 0;
            continue 'outer
          }
          Err(MbusError::InvalidFormat | MbusError::ChecksumMismatch) => {
            // Input is invalid but not incomplete,
            // so try advancing the buffer.
            self.buffer.remove(0);
            bytes_needed = 0;
            continue 'outer
          },
        }
      }
      bytes_needed = 0;

      match self.dlms.decrypt(&telegrams) {
        Ok((_, obis_map)) => {
          self.buffer.drain(0..telegrams_len);
          return Some(Ok(obis_map))
        },
        Err(DlmsError::Incomplete(n)) => {
          telegrams_needed += n.map(|t| t.get()).unwrap_or(1);
        },
        Err(DlmsError::InvalidFormat | DlmsError::ChecksumMismatch) => {
          // Other error, continue with next telegram.
          self.buffer.drain(0..telegram_1_len);
          telegrams_needed = 1;
          continue
        },
        Err(DlmsError::DecryptionFailed) => {
          return Some(Err(Error::DecryptionFailed))
        },
      }
    }
  }
}
