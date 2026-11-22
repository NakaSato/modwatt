//! Minimal blocking Modbus TCP client.
//!
//! A Modbus TCP → RS485 gateway (ShineMaster, USR-TCP232, etc.) maps the MBAP
//! "unit identifier" onto the RTU slave address on the serial bus, so one TCP
//! connection can reach every inverter on the RS485 line.

use std::fmt;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

#[derive(Debug)]
pub enum ModbusError {
    /// Could not open the TCP connection to the gateway.
    Connect(io::Error),
    /// No (complete) reply before the timeout, or the socket failed.
    Io(io::Error),
    /// The device or gateway answered with a Modbus exception code.
    Exception(u8),
    /// The reply was malformed.
    Protocol(String),
}

impl ModbusError {
    /// Gateway exceptions 0x0A / 0x0B mean "nobody answered on the serial side".
    pub fn is_gateway_no_response(&self) -> bool {
        matches!(self, ModbusError::Exception(0x0A | 0x0B))
    }
}

impl fmt::Display for ModbusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ModbusError::Connect(e) => write!(f, "connect failed: {e}"),
            ModbusError::Io(e) => write!(f, "I/O: {e}"),
            ModbusError::Exception(c) => write!(f, "exception 0x{c:02X} ({})", exception_name(*c)),
            ModbusError::Protocol(s) => write!(f, "protocol error: {s}"),
        }
    }
}

pub fn exception_name(code: u8) -> &'static str {
    match code {
        0x01 => "illegal function",
        0x02 => "illegal data address",
        0x03 => "illegal data value",
        0x04 => "slave device failure",
        0x05 => "acknowledge",
        0x06 => "slave device busy",
        0x0A => "gateway path unavailable",
        0x0B => "gateway target failed to respond",
        _ => "unknown",
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Table {
    /// Function 0x03
    Holding,
    /// Function 0x04
    Input,
}

pub struct ModbusTcp {
    addr: SocketAddr,
    timeout: Duration,
    stream: Option<TcpStream>,
    transaction_id: u16,
    /// Minimum quiet time on the RS485 bus between consecutive requests.
    gap: Duration,
    last_request_end: Option<Instant>,
}

impl ModbusTcp {
    pub fn new(host: &str, port: u16, timeout: Duration, gap: Duration) -> Result<Self, ModbusError> {
        let addr = (host, port)
            .to_socket_addrs()
            .map_err(ModbusError::Connect)?
            .next()
            .ok_or_else(|| {
                ModbusError::Connect(io::Error::new(io::ErrorKind::NotFound, "host did not resolve"))
            })?;
        let mut client = Self {
            addr,
            timeout,
            stream: None,
            transaction_id: 0,
            gap,
            last_request_end: None,
        };
        client.ensure_connected()?;
        Ok(client)
    }

    fn ensure_connected(&mut self) -> Result<&mut TcpStream, ModbusError> {
        if self.stream.is_none() {
            let connect_timeout = self.timeout.max(Duration::from_secs(2));
            let s = TcpStream::connect_timeout(&self.addr, connect_timeout).map_err(ModbusError::Connect)?;
            s.set_nodelay(true).ok();
            self.stream = Some(s);
        }
        Ok(self.stream.as_mut().unwrap())
    }

    /// Read `count` 16-bit registers from `unit` (FC 0x03 / 0x04).
    pub fn read(&mut self, unit: u8, table: Table, start: u16, count: u16) -> Result<Vec<u16>, ModbusError> {
        let function: u8 = match table {
            Table::Holding => 0x03,
            Table::Input => 0x04,
        };
        let mut req = vec![function];
        req.extend(start.to_be_bytes());
        req.extend(count.to_be_bytes());
        let pdu = self.transact(unit, &req)?;
        let byte_count = *pdu.get(1).unwrap_or(&0) as usize;
        if byte_count != count as usize * 2 || pdu.len() < 2 + byte_count {
            return Err(ModbusError::Protocol(format!(
                "expected {} data bytes, got {byte_count}",
                count * 2
            )));
        }
        Ok(pdu[2..2 + byte_count]
            .as_chunks::<2>()
            .0
            .iter()
            .map(|b| u16::from_be_bytes(*b))
            .collect())
    }

    /// Write one holding register (FC 0x06).
    #[allow(dead_code)] // used by the simulator tests; scanner is read-only
    pub fn write_single(&mut self, unit: u8, addr: u16, value: u16) -> Result<(), ModbusError> {
        let mut req = vec![0x06];
        req.extend(addr.to_be_bytes());
        req.extend(value.to_be_bytes());
        let pdu = self.transact(unit, &req)?;
        if pdu != req {
            return Err(ModbusError::Protocol("write echo mismatch".into()));
        }
        Ok(())
    }

    /// Send one request PDU and return the matching response PDU.
    ///
    /// Enforces the minimum bus gap between requests. After any I/O failure the
    /// socket is dropped and transparently reopened on the next call, so a late
    /// reply from a slow slave can never be mistaken for a later answer.
    fn transact(&mut self, unit: u8, req_pdu: &[u8]) -> Result<Vec<u8>, ModbusError> {
        if let Some(t) = self.last_request_end {
            let wait = (t + self.gap).saturating_duration_since(Instant::now());
            if !wait.is_zero() {
                std::thread::sleep(wait);
            }
        }
        let result = self.transact_inner(unit, req_pdu);
        self.last_request_end = Some(Instant::now());
        if matches!(result, Err(ModbusError::Io(_)) | Err(ModbusError::Protocol(_))) {
            self.stream = None;
        }
        result
    }

    fn transact_inner(&mut self, unit: u8, req_pdu: &[u8]) -> Result<Vec<u8>, ModbusError> {
        let function = req_pdu[0];
        self.transaction_id = self.transaction_id.wrapping_add(1);
        let tid = self.transaction_id;
        let timeout = self.timeout;

        let mut req = Vec::with_capacity(7 + req_pdu.len());
        req.extend(tid.to_be_bytes());
        req.extend(0u16.to_be_bytes()); // protocol id
        req.extend(((req_pdu.len() + 1) as u16).to_be_bytes());
        req.push(unit);
        req.extend_from_slice(req_pdu);

        let stream = self.ensure_connected()?;
        stream.write_all(&req).map_err(ModbusError::Io)?;

        let deadline = Instant::now() + timeout;
        loop {
            let header = read_exact_until(stream, 7, deadline)?;
            let rx_tid = u16::from_be_bytes([header[0], header[1]]);
            let len = u16::from_be_bytes([header[4], header[5]]) as usize;
            if !(2..=254).contains(&len) {
                return Err(ModbusError::Protocol(format!("bad MBAP length {len}")));
            }
            let pdu = read_exact_until(stream, len - 1, deadline)?;
            if rx_tid != tid {
                continue; // stale reply to an earlier request; skip it
            }
            if pdu[0] == function | 0x80 {
                return Err(ModbusError::Exception(*pdu.get(1).unwrap_or(&0)));
            }
            if pdu[0] != function {
                return Err(ModbusError::Protocol(format!("unexpected function 0x{:02X}", pdu[0])));
            }
            return Ok(pdu);
        }
    }
}

fn read_exact_until(stream: &mut TcpStream, n: usize, deadline: Instant) -> Result<Vec<u8>, ModbusError> {
    let mut buf = vec![0u8; n];
    let mut filled = 0;
    while filled < n {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(ModbusError::Io(io::Error::new(io::ErrorKind::TimedOut, "no response")));
        }
        stream.set_read_timeout(Some(remaining)).map_err(ModbusError::Io)?;
        match stream.read(&mut buf[filled..]) {
            Ok(0) => {
                return Err(ModbusError::Io(io::Error::new(
                    io::ErrorKind::ConnectionAborted,
                    "gateway closed the connection",
                )));
            }
            Ok(k) => filled += k,
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {
                return Err(ModbusError::Io(io::Error::new(io::ErrorKind::TimedOut, "no response")));
            }
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(ModbusError::Io(e)),
        }
    }
    Ok(buf)
}
