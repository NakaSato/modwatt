//! Find Modbus TCP gateways on the local network (e.g. a ShineMaster that got
//! its address from the router via DHCP).
//!
//! Sweeps a /24 subnet with parallel TCP connects to the Modbus port(s), then
//! sends one read to each open port to tell real Modbus gateways apart from
//! other devices that merely accept connections. Multi-port RS485 converters
//! often map COM2..COM4 to TCP 503..505, so those are tried too.
//!
//! Hosts that speak Modbus get one HTTP `GET /` on port 80; brand keywords in
//! the reply label the gateway (ShineMaster, Waveshare, Ebyte, Moxa, …).

use std::io::{Read, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpStream, UdpSocket};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::Duration;

use eframe::egui;

use crate::modbus::{ModbusError, ModbusTcp, Table};

#[derive(Clone, Debug)]
pub struct Found {
    pub ip: Ipv4Addr,
    pub port: u16,
    pub verdict: Verdict,
    /// Gateway brand guessed from its web page, if it has one.
    pub vendor: Option<&'static str>,
}

/// Ports tried on every host: the configured one first, then the usual
/// multi-port converter range.
pub fn ports_for(configured: u16) -> Vec<u16> {
    let mut v = vec![configured];
    v.extend((502..=505).filter(|p| *p != configured));
    v
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Unit 1 answered with data.
    Inverter,
    /// The host speaks Modbus (answered with an exception, e.g. 0x0B from a gateway).
    Gateway(u8),
    /// Port open but no Modbus reply.
    PortOpen,
}

impl Verdict {
    pub fn describe(&self) -> String {
        match self {
            Verdict::Inverter => "Modbus gateway, device at ID 1".into(),
            Verdict::Gateway(0x0B) | Verdict::Gateway(0x0A) => "Modbus gateway, nothing at ID 1".into(),
            Verdict::Gateway(c) => format!("Modbus device (exception 0x{c:02X})"),
            Verdict::PortOpen => "Port open, no Modbus reply".into(),
        }
    }
}

pub enum DiscoveryEvent {
    Progress(u32),
    Found(Found),
    Done,
}

/// This machine's LAN IPv4 address, i.e. the interface the router routes through.
/// UDP `connect` only selects a route; no packet is sent.
pub fn local_ipv4() -> Option<Ipv4Addr> {
    let sock = UdpSocket::bind("0.0.0.0:0").ok()?;
    sock.connect("192.0.2.1:9").ok()?; // TEST-NET-1, never actually contacted
    match sock.local_addr().ok()?.ip() {
        IpAddr::V4(ip) if !ip.is_loopback() && !ip.is_unspecified() => Some(ip),
        _ => None,
    }
}

/// "192.168.1." style prefix for the local /24.
pub fn local_prefix() -> Option<String> {
    local_ipv4().map(|ip| {
        let o = ip.octets();
        format!("{}.{}.{}.", o[0], o[1], o[2])
    })
}

/// Parse "192.168.1." / "192.168.1" / "192.168.1.0" into the first three octets.
pub fn parse_prefix(s: &str) -> Option<[u8; 3]> {
    let parts: Vec<&str> = s.trim().trim_end_matches('.').split('.').collect();
    if !(3..=4).contains(&parts.len()) {
        return None;
    }
    let mut o = [0u8; 3];
    for (i, p) in parts.iter().take(3).enumerate() {
        o[i] = p.parse().ok()?;
    }
    Some(o)
}

pub fn spawn(prefix: [u8; 3], ports: Vec<u16>, ctx: egui::Context) -> Receiver<DiscoveryEvent> {
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        let (hit_tx, hit_rx) = mpsc::channel();
        let workers: Vec<_> = (1..=254u8)
            .map(|last| {
                let (hit_tx, ports) = (hit_tx.clone(), ports.clone());
                thread::spawn(move || {
                    let ip = Ipv4Addr::new(prefix[0], prefix[1], prefix[2], last);
                    let open: Vec<u16> = ports
                        .into_iter()
                        .filter(|&port| {
                            let addr = SocketAddr::from((ip, port));
                            TcpStream::connect_timeout(&addr, Duration::from_millis(400)).is_ok()
                        })
                        .collect();
                    let _ = hit_tx.send((ip, open));
                })
            })
            .collect();
        drop(hit_tx);

        let mut open = Vec::new();
        for (n, (ip, ports)) in hit_rx.iter().enumerate() {
            open.extend(ports.into_iter().map(|port| (ip, port)));
            let _ = tx.send(DiscoveryEvent::Progress(n as u32 + 1));
            ctx.request_repaint();
        }
        workers.into_iter().for_each(|w| {
            let _ = w.join();
        });

        open.sort();
        let mut vendors = std::collections::HashMap::new();
        for (ip, port) in open {
            let verdict = classify(ip, port);
            let vendor = if verdict == Verdict::PortOpen {
                None
            } else {
                *vendors.entry(ip).or_insert_with(|| http_vendor(ip))
            };
            let _ = tx.send(DiscoveryEvent::Found(Found { ip, port, verdict, vendor }));
            ctx.request_repaint();
        }
        let _ = tx.send(DiscoveryEvent::Done);
        ctx.request_repaint();
    });
    rx
}

fn classify(ip: Ipv4Addr, port: u16) -> Verdict {
    let Ok(mut c) = ModbusTcp::new(&ip.to_string(), port, Duration::from_millis(800), Duration::ZERO) else {
        return Verdict::PortOpen;
    };
    match c.read(1, Table::Holding, 0, 1) {
        Ok(_) => Verdict::Inverter,
        Err(ModbusError::Exception(code)) => Verdict::Gateway(code),
        Err(_) => Verdict::PortOpen,
    }
}

/// Fetch the gateway's web page (if any) and look for a brand name in it.
fn http_vendor(ip: Ipv4Addr) -> Option<&'static str> {
    let timeout = Duration::from_millis(800);
    let mut s = TcpStream::connect_timeout(&SocketAddr::from((ip, 80)), timeout).ok()?;
    s.set_read_timeout(Some(timeout)).ok()?;
    s.set_write_timeout(Some(timeout)).ok()?;
    s.write_all(format!("GET / HTTP/1.0\r\nHost: {ip}\r\nConnection: close\r\n\r\n").as_bytes()).ok()?;
    let mut buf = Vec::new();
    let _ = s.take(16 * 1024).read_to_end(&mut buf); // a timeout still leaves what arrived
    vendor_from_http(&String::from_utf8_lossy(&buf))
}

/// Brand keywords as they appear in server headers, titles or page text.
pub fn vendor_from_http(page: &str) -> Option<&'static str> {
    let page = page.to_ascii_lowercase();
    const BRANDS: [(&str, &str); 10] = [
        ("shinemaster", "Growatt ShineMaster"),
        ("shinewilan", "Growatt ShineWiLan"),
        ("growatt", "Growatt"),
        ("waveshare", "Waveshare"),
        ("cdebyte", "Ebyte"),
        ("ebyte", "Ebyte"),
        ("mgate", "Moxa MGate"),
        ("moxa", "Moxa"),
        ("usr-tcp232", "USR IOT"),
        ("elfin", "Hi-Flying Elfin"),
    ];
    BRANDS.iter().find(|(k, _)| page.contains(k)).map(|(_, v)| *v)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_prefixes() {
        assert_eq!(parse_prefix("192.168.1."), Some([192, 168, 1]));
        assert_eq!(parse_prefix("10.0.5"), Some([10, 0, 5]));
        assert_eq!(parse_prefix("10.0.5.0"), Some([10, 0, 5]));
        assert_eq!(parse_prefix("10.0"), None);
        assert_eq!(parse_prefix("300.0.0."), None);
    }

    #[test]
    fn finds_simulator_on_loopback() {
        use crate::sim::{SimOptions, Simulator};
        let sim = Simulator::start(SimOptions { port: 0, ..Default::default() }).unwrap();
        let rx = spawn([127, 0, 0], ports_for(sim.port), egui::Context::default());
        let found: Vec<Found> = rx
            .iter()
            .take_while(|e| !matches!(e, DiscoveryEvent::Done))
            .filter_map(|e| match e {
                DiscoveryEvent::Found(f) => Some(f),
                _ => None,
            })
            .collect();
        assert!(
            found.iter().any(|f| f.ip == Ipv4Addr::LOCALHOST && f.port == sim.port && f.verdict == Verdict::Inverter),
            "{found:?}"
        );
    }

    #[test]
    fn ports_include_converter_range() {
        assert_eq!(ports_for(502), vec![502, 503, 504, 505]);
        assert_eq!(ports_for(8899), vec![8899, 502, 503, 504, 505]);
    }

    #[test]
    fn recognises_gateway_brands() {
        assert_eq!(vendor_from_http("HTTP/1.0 200 OK\r\n\r\n<title>ShineMaster</title>"), Some("Growatt ShineMaster"));
        assert_eq!(vendor_from_http("Server: Moxa MGate MB3170"), Some("Moxa MGate"));
        assert_eq!(vendor_from_http("<title>Waveshare RS485 TO ETH</title>"), Some("Waveshare"));
        assert_eq!(vendor_from_http("<html>router</html>"), None);
    }
}
