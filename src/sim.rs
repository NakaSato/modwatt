//! Simulated ShineMaster / RS485 gateway with Growatt devices behind it.
//!
//! Listens for Modbus TCP and answers per unit ID as if the request had been
//! forwarded onto an RS485 bus. Implements the same register maps as
//! `growatt.rs` and `meter.rs` (keep them in sync), plus FC 0x06 writes to
//! On/Off (holding 0) and active power limit (holding 3) on inverters.
//!
//! Each device exposes a set of register blocks per function code. A read
//! must fall entirely inside one block, or it gets exception 0x02 like a real
//! slave asked for an unmapped range. Meters therefore reject the Growatt
//! holding 0–124 read.

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// MID/MAX-style grid-tie: 10-char serial in holding 23..=27.
    Mid,
    /// MOD TL3-XH hybrid: 23..=27 blank, 30-char serial in holding 3001..=3015.
    ModXh,
    /// SPH hybrid with battery (storage block at input 1000+).
    Sph,
    /// SPF off-grid inverter with battery.
    Spf,
    /// Eastron SDM630 three-phase meter (Growatt TPM-E).
    Sdm630,
    /// Chint DTSU666 three-phase meter (Growatt TPM-C).
    Dtsu666,
    /// Eastron SDM120 single-phase meter (Growatt SPM-E).
    Sdm120,
    /// Chint DDSU666 single-phase meter (Growatt SPM-C).
    Ddsu666,
}

impl Model {
    pub const ALL: [Model; 8] = [
        Model::Mid,
        Model::ModXh,
        Model::Sph,
        Model::Spf,
        Model::Sdm630,
        Model::Dtsu666,
        Model::Sdm120,
        Model::Ddsu666,
    ];

    pub fn is_meter(self) -> bool {
        matches!(self, Model::Sdm630 | Model::Dtsu666 | Model::Sdm120 | Model::Ddsu666)
    }
}

impl Model {
    pub fn hint(self) -> &'static str {
        match self {
            Model::Mid => "MID/MAX style: 10-char serial at holding 23–27",
            Model::ModXh => "MOD TL3-XH style: 30-char serial at holding 3001–3015",
            Model::Sph => "SPH hybrid: battery in input 1000+",
            Model::Spf => "SPF off-grid: battery and load in input 0+",
            Model::Sdm630 => "Eastron SDM630 meter: float input registers",
            Model::Dtsu666 => "Chint DTSU666 meter: float holding 0x2000+",
            Model::Sdm120 => "Eastron SDM120 meter: 1-phase float input registers",
            Model::Ddsu666 => "Chint DDSU666 meter: 1-phase float holding 0x2000+",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Model::Mid => "MID",
            Model::ModXh => "MOD-XH",
            Model::Sph => "SPH",
            Model::Spf => "SPF",
            Model::Sdm630 => "SDM630",
            Model::Dtsu666 => "DTSU666",
            Model::Sdm120 => "SDM120",
            Model::Ddsu666 => "DDSU666",
        }
    }
}

#[derive(Clone, Debug)]
pub struct SimInverter {
    pub unit: u8,
    pub model: Model,
    pub rated_kw: f64,
    pub fault: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EmptyIdReply {
    /// Reply with exception 0x0B like most gateways do.
    GatewayException,
    /// Never reply; forces the client to time out.
    Silent,
}

#[derive(Clone, Debug)]
pub struct SimOptions {
    /// 0 picks a free port.
    pub port: u16,
    pub inverters: Vec<SimInverter>,
    pub empty_id_reply: EmptyIdReply,
    /// Delay replies as if they travelled over RS485 at this baud rate.
    pub serial_baud: Option<u32>,
}

impl Default for SimOptions {
    fn default() -> Self {
        let inv = |unit, model, rated_kw, fault| SimInverter { unit, model, rated_kw, fault };
        Self {
            port: 5020,
            inverters: vec![
                inv(1, Model::Mid, 50.0, false),
                inv(2, Model::Mid, 50.0, false),
                inv(3, Model::Mid, 30.0, false),
                inv(4, Model::Sph, 10.0, false),
                inv(5, Model::Spf, 5.0, false),
                inv(7, Model::ModXh, 10.0, false),
                inv(12, Model::Mid, 50.0, true),
                // Real meter default IDs are 1–4; high IDs avoid clashing with the inverters.
                inv(20, Model::Sdm630, 0.0, false),
                inv(21, Model::Dtsu666, 0.0, false),
                inv(22, Model::Sdm120, 0.0, false),
                inv(23, Model::Ddsu666, 0.0, false),
            ],
            empty_id_reply: EmptyIdReply::GatewayException,
            serial_baud: None,
        }
    }
}

pub struct Simulator {
    pub port: u16,
    requests: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl Simulator {
    pub fn start(opts: SimOptions) -> io::Result<Self> {
        let listener = TcpListener::bind(("127.0.0.1", opts.port))?;
        listener.set_nonblocking(true)?;
        let port = listener.local_addr()?.port();
        let stop = Arc::new(AtomicBool::new(false));
        let requests = Arc::new(AtomicU64::new(0));
        let bus = Arc::new(Bus::new(&opts));

        let handle = {
            let (stop, requests) = (stop.clone(), requests.clone());
            thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    match listener.accept() {
                        Ok((stream, _)) => {
                            let (bus, stop, requests) = (bus.clone(), stop.clone(), requests.clone());
                            thread::spawn(move || {
                                let _ = serve(stream, &bus, &stop, &requests);
                            });
                        }
                        Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                            thread::sleep(Duration::from_millis(20))
                        }
                        Err(_) => thread::sleep(Duration::from_millis(100)),
                    }
                }
            })
        };
        Ok(Self { port, requests, stop, handle: Some(handle) })
    }

    pub fn requests(&self) -> u64 {
        self.requests.load(Ordering::Relaxed)
    }
}

impl Drop for Simulator {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

// ---------------------------------------------------------------------------

struct Device {
    cfg: SimInverter,
    holding: [u16; 125],
    holding_ext: [u16; 125], // 3000..=3124
    base_total_kwh: f64,
}

struct Bus {
    /// One mutex = one RS485 bus: only one request is on the wire at a time.
    devices: Mutex<Vec<Device>>,
    empty_id_reply: EmptyIdReply,
    serial_baud: Option<u32>,
    started: Instant,
}

impl Bus {
    fn new(opts: &SimOptions) -> Self {
        let devices = opts
            .inverters
            .iter()
            .map(|cfg| {
                let mut holding = [0u16; 125];
                let mut holding_ext = [0u16; 125];
                holding[0] = 1; // on
                holding[3] = 100; // 100 % active power
                put_ascii(&mut holding[9..12], "GH1.0A");
                holding[22] = 0; // 9600 baud
                holding[30] = cfg.unit as u16;
                // DTC (43) and trackers/phases (44). Illustrative values only.
                (holding[43], holding[44]) = match cfg.model {
                    Model::Mid => (0x0800, 0x0203),
                    Model::ModXh => (5400, 0x0203),
                    Model::Sph => (0x0C00, 0x0201),
                    Model::Spf => (0x0D00, 0x0101),
                    _ => (0, 0),
                };
                let short_sn = format!("SIM{:07}", cfg.unit as u32 * 1111);
                match cfg.model {
                    Model::Mid | Model::Sph | Model::Spf => put_ascii(&mut holding[23..28], &short_sn),
                    Model::ModXh => {
                        let long_sn = format!("SIMXH{:025}", cfg.unit as u64 * 123_456_789);
                        put_ascii(&mut holding_ext[1..16], &long_sn);
                    }
                    Model::Sdm630 | Model::Dtsu666 | Model::Sdm120 | Model::Ddsu666 => {}
                }
                Device { cfg: cfg.clone(), holding, holding_ext, base_total_kwh: 1000.0 + cfg.unit as f64 * 37.5 }
            })
            .collect();
        Self {
            devices: Mutex::new(devices),
            empty_id_reply: opts.empty_id_reply,
            serial_baud: opts.serial_baud,
            started: Instant::now(),
        }
    }

    /// Returns the response PDU, or None for "no reply".
    fn handle(&self, unit: u8, pdu: &[u8]) -> Option<Vec<u8>> {
        let mut devices = self.devices.lock().unwrap();
        let Some(dev) = devices.iter_mut().find(|d| d.cfg.unit == unit) else {
            drop(devices);
            return match self.empty_id_reply {
                EmptyIdReply::GatewayException => Some(vec![pdu.first().copied().unwrap_or(0) | 0x80, 0x0B]),
                EmptyIdReply::Silent => None,
            };
        };
        let resp = respond(dev, pdu, self.started.elapsed().as_secs_f64());
        if let Some(baud) = self.serial_baud {
            // RTU frame = unit + PDU + CRC; ~10 bits per byte; plus turnaround.
            let bytes = (pdu.len() + 3) + (resp.len() + 3);
            thread::sleep(Duration::from_secs_f64(bytes as f64 * 10.0 / baud as f64) + Duration::from_millis(5));
        }
        Some(resp)
    }
}

fn respond(dev: &mut Device, pdu: &[u8], t: f64) -> Vec<u8> {
    let exc = |f: u8, code: u8| vec![f | 0x80, code];
    let func = pdu[0];
    if pdu.len() < 5 {
        return exc(func, 0x03);
    }
    let a = u16::from_be_bytes([pdu[1], pdu[2]]);
    let b = u16::from_be_bytes([pdu[3], pdu[4]]);
    match func {
        0x03 | 0x04 => {
            if !(1..=125).contains(&b) {
                return exc(func, 0x03);
            }
            let Some(regs) = read_regs(dev, func, a, b, t) else {
                return exc(func, 0x02);
            };
            let mut out = vec![func, (b * 2) as u8];
            regs.iter().for_each(|r| out.extend(r.to_be_bytes()));
            out
        }
        0x06 if dev.cfg.model.is_meter() => exc(func, 0x02),
        0x06 => match a {
            0 if b <= 1 => {
                dev.holding[0] = b;
                pdu.to_vec()
            }
            3 if b <= 100 || b == 255 => {
                dev.holding[3] = b;
                pdu.to_vec()
            }
            0 | 3 => exc(func, 0x03),
            _ => exc(func, 0x02),
        },
        _ => exc(func, 0x01),
    }
}

/// A read must lie entirely inside one mapped block.
fn read_regs(dev: &Device, func: u8, start: u16, count: u16, t: f64) -> Option<Vec<u16>> {
    let (start, end) = (start as u32, start as u32 + count as u32);
    blocks(dev, func, t).into_iter().find_map(|(base, regs)| {
        let base = base as u32;
        (start >= base && end <= base + regs.len() as u32).then(|| regs[(start - base) as usize..(end - base) as usize].to_vec())
    })
}

/// Register blocks per model and function code (0x03 holding, 0x04 input).
fn blocks(dev: &Device, func: u8, t: f64) -> Vec<(u16, Vec<u16>)> {
    use Model::*;
    match (dev.cfg.model, func) {
        (Sdm630 | Sdm120, 0x04) => vec![(0x0000, eastron_input(dev, t))],
        (Sdm630 | Sdm120, 0x03) => {
            let sn = 20_000_000 + dev.cfg.unit as u32;
            vec![(0xFC00, vec![(sn >> 16) as u16, sn as u16])]
        }
        (Dtsu666, 0x03) => {
            let (live, energy) = dtsu_blocks(dev, t);
            vec![(0x2000, live), (0x1000, energy)]
        }
        (Ddsu666, 0x03) => {
            let (live, energy) = ddsu_blocks(dev, t);
            vec![(0x2000, live), (0x4000, energy)]
        }
        (Sdm630 | Sdm120 | Dtsu666 | Ddsu666, _) => vec![],
        (_, 0x03) => vec![(0, dev.holding.to_vec()), (3000, dev.holding_ext.to_vec())],
        (Spf, 0x04) => vec![(0, spf_input(dev, t))],
        (Sph, 0x04) => vec![(0, grid_input(dev, t)), (1000, sph_storage(dev, t))],
        (ModXh, 0x04) => vec![(0, grid_input(dev, t)), (3000, vec![0; 125]), (3125, xh_storage(dev, t))],
        (Mid, 0x04) => vec![(0, grid_input(dev, t))],
        _ => vec![],
    }
}

// --- inverters --------------------------------------------------------------

fn sun(dev: &Device, t: f64) -> f64 {
    let u = dev.cfg.unit as f64;
    (0.65 + 0.25 * (t / 90.0 + u).sin() + 0.08 * (t / 7.3 + u * 2.0).sin()).clamp(0.0, 1.0)
}

/// Inverter running state: 0 waiting/off, 1 normal, 3 fault.
fn run_state(dev: &Device) -> u16 {
    if dev.cfg.fault {
        3
    } else if dev.holding[0] != 1 {
        0
    } else {
        1
    }
}

/// Drifting battery: SOC in 15–85 %, power signed (+ charging).
fn battery(dev: &Device, t: f64) -> (u16, f64) {
    let phase = t / 120.0 + dev.cfg.unit as f64;
    let soc = (50.0 + 35.0 * phase.sin()).round() as u16;
    let power = if run_state(dev) == 1 { dev.cfg.rated_kw * 1000.0 * 0.4 * phase.cos() } else { 0.0 };
    (soc, power)
}

/// Grid-tie base input map 0..=124 (also used by SPH and MOD-XH).
fn grid_input(dev: &Device, t: f64) -> Vec<u16> {
    let mut r = vec![0u16; 125];
    let limit = match dev.holding[3] {
        255 => 1.0,
        p => p as f64 / 100.0,
    };
    let u = dev.cfg.unit as f64;
    let sun = sun(dev, t);
    let rated_w = dev.cfg.rated_kw * 1000.0;
    let status = run_state(dev);
    let pv_w = if status == 1 { rated_w * sun } else { 0.0 };
    let ac_w = (pv_w * 0.975).min(rated_w * limit);
    let today_kwh = if dev.cfg.fault { 0.0 } else { rated_w * 0.6 * t / 3.6e6 + u * 0.8 };

    r[0] = status;
    put_u32(&mut r, 1, (pv_w * 10.0) as u32);
    r[3] = if status == 1 { (6200.0 + 300.0 * sun) as u16 } else { 0 }; // PV1 V, 0.1 V
    put_u32(&mut r, 35, (ac_w * 10.0) as u32);
    r[37] = 5000 + ((t / 5.0).sin() * 3.0) as i16 as u16; // Fac, 0.01 Hz
    r[38] = 2300; // Vac1, 0.1 V
    for (i, reg) in r[50..=52].iter_mut().enumerate() {
        *reg = 4000 + i as u16 * 3; // ~400 V line voltage, 0.1 V
    }
    put_u32(&mut r, 53, (today_kwh * 10.0) as u32);
    put_u32(&mut r, 55, ((dev.base_total_kwh + today_kwh) * 10.0) as u32);
    r
}

/// SPH/SPA storage block, input 1000..=1124.
fn sph_storage(dev: &Device, t: f64) -> Vec<u16> {
    let mut r = vec![0u16; 125];
    let (soc, p) = battery(dev, t);
    r[0] = match run_state(dev) {
        1 => 5, // PV on-grid
        s => s,
    };
    put_u32(&mut r, 9, ((-p).max(0.0) * 10.0) as u32); // 1009 discharge
    put_u32(&mut r, 11, (p.max(0.0) * 10.0) as u32); // 1011 charge
    r[13] = (512.0 + soc as f64 * 0.4) as u16; // 1013 Vbat, 0.1 V (51–55 V)
    r[14] = soc; // 1014
    r
}

/// TL-XH storage block, input 3125..=3249.
fn xh_storage(dev: &Device, t: f64) -> Vec<u16> {
    let mut r = vec![0u16; 125];
    let (soc, p) = battery(dev, t);
    let v = 400.0 + soc as f64 * 0.5;
    r[3169 - 3125] = (v * 100.0) as u16; // 0.01 V
    r[3170 - 3125] = (p.abs() / v * 10.0) as u16; // 0.1 A
    r[3171 - 3125] = soc;
    put_u32(&mut r, 3178 - 3125, ((-p).max(0.0) * 10.0) as u32);
    put_u32(&mut r, 3180 - 3125, (p.max(0.0) * 10.0) as u32);
    r
}

/// SPF off-grid input map 0..=124.
fn spf_input(dev: &Device, t: f64) -> Vec<u16> {
    let mut r = vec![0u16; 125];
    let u = dev.cfg.unit as f64;
    let rated_w = dev.cfg.rated_kw * 1000.0;
    let state = run_state(dev);
    let (soc, p) = battery(dev, t);
    let pv_w = if state == 1 { rated_w * sun(dev, t) } else { 0.0 };
    let load_w = if state == 1 { rated_w * (0.3 + 0.1 * (t / 11.0 + u).sin()) } else { 0.0 };
    let today_kwh = rated_w * 0.4 * t / 3.6e6 + u * 0.5;

    r[0] = match state {
        1 if p > 0.0 => 5, // PV charging
        1 => 2,            // battery discharging
        s => s,            // 0 standby, 3 fault
    };
    r[1] = if state == 1 { 3500 } else { 0 }; // Vpv1, 0.1 V
    put_u32(&mut r, 3, (pv_w * 10.0) as u32);
    put_u32(&mut r, 9, (load_w * 10.0) as u32);
    put_u32(&mut r, 13, (p.max(0.0) * 10.0) as u32);
    r[17] = ((48.0 + soc as f64 * 0.06) * 100.0) as u16; // 0.01 V
    r[18] = soc;
    r[20] = 2300; // grid V
    r[21] = 5000; // line frequency, 0.01 Hz
    r[22] = 2300; // output V
    r[23] = 5000; // output frequency
    r[25] = 350; // temperature, 0.1 °C
    r[27] = (load_w / rated_w.max(1.0) * 1000.0) as u16; // load, 0.1 %
    put_u32(&mut r, 48, (today_kwh * 10.0) as u32);
    put_u32(&mut r, 50, ((dev.base_total_kwh + today_kwh) * 10.0) as u32);
    put_u32(&mut r, 73, ((-p).max(0.0) * 10.0) as u32);
    put_u32(&mut r, 77, (-p * 10.0) as i32 as u32); // + discharge, - charge
    r
}

// --- meters -----------------------------------------------------------------

/// Grid power at the meter, + import / - export, plus cumulative energies.
fn grid(dev: &Device, t: f64) -> (f64, f64, f64, f64) {
    let u = dev.cfg.unit as f64;
    let w = 1800.0 * (t / 60.0 + u).sin() - 300.0;
    let hz = 50.0 + 0.02 * (t / 9.0).sin();
    let import = 5000.0 + u * 10.0 + t / 3600.0;
    let export = 1200.0 + u * 5.0 + t / 7200.0;
    (w, hz, import, export)
}

fn eastron_input(dev: &Device, t: f64) -> Vec<u16> {
    let mut r = vec![0u16; 0x100];
    let (w, hz, import, export) = grid(dev, t);
    let three = dev.cfg.model == Model::Sdm630;
    let phases: &[f64] = if three { &[230.1, 231.4, 229.8] } else { &[230.6] };
    for (i, v) in phases.iter().enumerate() {
        put_f32(&mut r, i * 2, *v); // 0x0000.. V
        put_f32(&mut r, 0x06 + i * 2, w / phases.len() as f64 / v); // current
        put_f32(&mut r, 0x0C + i * 2, w / phases.len() as f64); // P per phase
    }
    if three {
        put_f32(&mut r, 0x34, w); // total system power (not on SDM120)
    }
    put_f32(&mut r, 0x46, hz);
    put_f32(&mut r, 0x48, import);
    put_f32(&mut r, 0x4A, export);
    r
}

fn dtsu_blocks(dev: &Device, t: f64) -> (Vec<u16>, Vec<u16>) {
    let (w, hz, import, export) = grid(dev, t);
    let mut live = vec![0u16; 0x46];
    for (i, v) in [3985.0, 3992.0, 3979.0].iter().enumerate() {
        put_f32(&mut live, i * 2, *v); // Uab.. ×0.1 V
    }
    for (i, v) in [2301.0, 2314.0, 2298.0].iter().enumerate() {
        put_f32(&mut live, 0x06 + i * 2, *v); // Ua.. ×0.1 V
        put_f32(&mut live, 0x0C + i * 2, w / 3.0 / (v / 10.0) * 1000.0); // I ×0.001 A
        put_f32(&mut live, 0x14 + i * 2, w / 3.0 * 10.0); // P per phase ×0.1 W
    }
    put_f32(&mut live, 0x12, w * 10.0); // Pt ×0.1 W
    put_f32(&mut live, 0x44, hz * 100.0); // Freq ×0.01 Hz
    let mut energy = vec![0u16; 0x32];
    put_f32(&mut energy, 0x1E, import);
    put_f32(&mut energy, 0x28, export);
    (live, energy)
}

fn ddsu_blocks(dev: &Device, t: f64) -> (Vec<u16>, Vec<u16>) {
    let (w, hz, import, export) = grid(dev, t);
    let v = 230.4;
    let mut live = vec![0u16; 0x10];
    put_f32(&mut live, 0x00, v);
    put_f32(&mut live, 0x02, w.abs() / v);
    put_f32(&mut live, 0x04, w / 1000.0); // kW
    put_f32(&mut live, 0x08, w.abs() / 1000.0); // S, kVA
    put_f32(&mut live, 0x0A, 0.99); // PF
    put_f32(&mut live, 0x0E, hz);
    let mut energy = vec![0u16; 0x0C];
    put_f32(&mut energy, 0x00, import);
    put_f32(&mut energy, 0x0A, export);
    (live, energy)
}

fn put_f32(r: &mut [u16], i: usize, v: f64) {
    put_u32(r, i, (v as f32).to_bits());
}

fn put_u32(r: &mut [u16], i: usize, v: u32) {
    r[i] = (v >> 16) as u16;
    r[i + 1] = v as u16;
}

fn put_ascii(regs: &mut [u16], s: &str) {
    let b = s.as_bytes();
    for (i, reg) in regs.iter_mut().enumerate() {
        let hi = *b.get(i * 2).unwrap_or(&0);
        let lo = *b.get(i * 2 + 1).unwrap_or(&0);
        *reg = u16::from_be_bytes([hi, lo]);
    }
}

fn serve(mut s: TcpStream, bus: &Bus, stop: &AtomicBool, requests: &AtomicU64) -> io::Result<()> {
    s.set_nonblocking(false)?;
    s.set_nodelay(true)?;
    s.set_read_timeout(Some(Duration::from_millis(100)))?;
    let mut buf = Vec::new();
    let mut chunk = [0u8; 512];
    while !stop.load(Ordering::Relaxed) {
        match s.read(&mut chunk) {
            Ok(0) => return Ok(()),
            Ok(n) => buf.extend_from_slice(&chunk[..n]),
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => continue,
            Err(e) => return Err(e),
        }
        // Process every complete MBAP frame in the buffer.
        while buf.len() >= 7 {
            let len = u16::from_be_bytes([buf[4], buf[5]]) as usize;
            if !(2..=254).contains(&len) {
                return Ok(()); // garbage: drop the connection like a real gateway
            }
            if buf.len() < 6 + len {
                break;
            }
            let frame: Vec<u8> = buf.drain(..6 + len).collect();
            requests.fetch_add(1, Ordering::Relaxed);
            let unit = frame[6];
            if let Some(pdu) = bus.handle(unit, &frame[7..]) {
                let mut resp = frame[0..4].to_vec();
                resp.extend(((pdu.len() + 1) as u16).to_be_bytes());
                resp.push(unit);
                resp.extend(pdu);
                s.write_all(&resp)?;
            }
        }
    }
    Ok(())
}
