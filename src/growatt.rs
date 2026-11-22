//! Growatt register maps and the per-address `probe()` that identifies what
//! answered: Growatt grid-tie, hybrid or off-grid inverter, a rebadged energy
//! meter (see `meter.rs`), or something unknown.
//!
//! Sources:
//! * [P2] Growatt Modbus RTU Protocol II V1.39 as transcribed by the
//!   Growatt_ModbusTCP project:
//!   <https://0xaha.github.io/Growatt_ModbusTCP/developer/protocol-v139/>
//! * [OG] Growatt Off-Grid protocol V0.26, same project:
//!   <https://0xaha.github.io/Growatt_ModbusTCP/developer/protocol-offgrid/>
//! * [HA] WouterTuinstra/Homeassistant-Growatt-Local-Modbus, `API/device_type/*.py`:
//!   <https://github.com/WouterTuinstra/Homeassistant-Growatt-Local-Modbus>
//!
//! ## Shared base map (MIC/MIN/MOD/MID/MAX/SPH/SPA, also SPF)
//! Holding (FC 0x03). Verified against [HA] for 9, 23, 43 and 44, and against [OG] for 30.
//!   0       On/Off
//!   9..=11  Firmware version (ASCII, 6 chars)
//!   22      RS485 baud rate (0 = 9600, 1 = 38400). Unverified.
//!   23..=27 Serial number    (ASCII, 10 chars)
//!   30      RS485 COM address (slave ID configured on the inverter)
//!   43      DTC device type code (raw; family table below is only a hint)
//!   44      trackers (high byte) / phases (low byte)
//! Newer models carry a 30-char serial at 3001..=3015 (TL-X/TL-XH, per [HA]) or
//! at 133..=147 (some TL3-X; unverified). Those are tried only when 23..=27 is blank.
//!
//! Grid-tie input (FC 0x04). Verified against [HA] `inverter_120.py`.
//!   0       status (0 waiting, 1 normal, 3 fault)
//!   1..=2   PV power (u32, 0.1 W)
//!   35..=36 AC output power (u32, 0.1 W)
//!   37      grid frequency (0.01 Hz)
//!   53..=54 / 55..=56 energy today / total (u32, 0.1 kWh)
//!
//! ## Hybrid storage blocks (input registers)
//! SPH/SPA, 1000..=1124. Verified against [P2] and [HA] `storage_120.py`:
//!   1000 work mode, 1009..=1010 discharge (u32 0.1 W), 1011..=1012 charge (u32 0.1 W),
//!   1013 Vbat (0.1 V), 1014 SOC (%).
//! TL-XH/MOD-XH (and WIT, assumed to use the same TL-XH layout, unverified), 3125..=3249.
//! Verified against [P2] and [HA]:
//!   3169 Vbat (0.01 V), 3170 Ibat, 3171 SOC (%), 3178..=3179 discharge, 3180..=3181 charge
//!   (u32 0.1 W).
//! [P2] also lists WIT battery registers at 8000..=8139. Those are not read here.
//!
//! ## Off-grid SPF input map. Verified against [OG] and [HA] `offgrid.py`.
//!   0 status (see `spf_status_text`), 3..=4 / 5..=6 Ppv1 / Ppv2 (u32 0.1 W),
//!   9..=10 output active power (u32 0.1 W), 17 Vbat (0.01 V), 18 SOC (%),
//!   21 line frequency (0.01 Hz), 23 output frequency (0.01 Hz),
//!   48..=51 / 52..=55 PV1 / PV2 energy today/total (u32 0.1 kWh, scale per [HA] default),
//!   77..=78 battery power (i32 0.1 W, positive = discharge).
//!
//! ## Classification
//! Kinds are decided by register signatures (SPF input layout, storage block
//! answering), not by DTC. See `probe()` for the request budget.

use crate::meter;
use crate::modbus::{ModbusError, ModbusTcp, Table};

/// What kind of device answered at a slave address.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DeviceKind {
    /// On-grid PV inverter (MIC/MIN/MOD/MID/MAX TL-X/TL3-X).
    #[default]
    GridTie,
    /// Hybrid / storage inverter (SPH, SPA, MIN-XH, MOD-XH). Battery data
    /// (ARK/APX packs) is read through the inverter's BMS registers.
    Hybrid,
    /// Off-grid inverter (SPF series).
    OffGrid,
    /// Energy meter on the bus (Growatt SPM/TPM are rebadged Eastron/Chint).
    Meter(MeterKind),
    /// Answered Modbus but matched no known profile.
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeterKind {
    /// Eastron SDM120/SDM230/SDM630 (Growatt SPM / TPM-E): IEEE754 floats, FC 0x04.
    EastronSdm,
    /// Chint DTSU666 (Growatt TPM-C): floats, FC 0x03 at 0x2000/0x101E.
    ChintDtsu,
    /// Chint DDSU666 (Growatt SPM-C), single-phase: floats, FC 0x03 at 0x2000/0x4000.
    ChintDdsu,
}

impl DeviceKind {
    pub fn label(self) -> &'static str {
        match self {
            DeviceKind::GridTie => "Inverter",
            DeviceKind::Hybrid => "Hybrid",
            DeviceKind::OffGrid => "Off-grid",
            DeviceKind::Meter(MeterKind::EastronSdm) => "Meter (Eastron)",
            DeviceKind::Meter(MeterKind::ChintDtsu) => "Meter (Chint)",
            DeviceKind::Meter(MeterKind::ChintDdsu) => "Meter (Chint 1-ph)",
            DeviceKind::Unknown => "Unknown",
        }
    }

    pub fn is_meter(self) -> bool {
        matches!(self, DeviceKind::Meter(_))
    }
}

/// Battery data read through a hybrid or off-grid inverter.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BatteryInfo {
    pub soc_pct: Option<f64>,
    pub voltage_v: Option<f64>,
    /// Positive while charging.
    pub charge_w: Option<f64>,
    pub discharge_w: Option<f64>,
}

/// Energy-meter reading.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeterInfo {
    /// L1 (or only) phase voltage.
    pub voltage_v: Option<f64>,
    /// Total active power; positive = import from grid, negative = export.
    pub power_w: Option<f64>,
    pub frequency_hz: Option<f64>,
    pub import_kwh: Option<f64>,
    pub export_kwh: Option<f64>,
}

#[derive(Clone, Debug, Default)]
pub struct DeviceInfo {
    pub unit: u8,
    pub kind: DeviceKind,
    /// Growatt holding 43 "DTC" device type code, shown raw.
    pub device_type_code: Option<u16>,
    pub serial: Option<String>,
    pub firmware: Option<String>,
    /// Slave address the device itself reports (Growatt holding 30).
    pub com_address: Option<u16>,
    pub baud: Option<u32>,
    pub status: Option<u16>,
    pub pv_power_w: Option<f64>,
    /// Inverter AC output (for off-grid: load / output power).
    pub ac_power_w: Option<f64>,
    pub energy_today_kwh: Option<f64>,
    pub energy_total_kwh: Option<f64>,
    pub battery: Option<BatteryInfo>,
    pub meter: Option<MeterInfo>,
    /// Short probe-fingerprint lines ("holding 0–124: exception 0x02", "DTC 5400", …),
    /// so an unrecognised device can still be identified by hand.
    pub notes: Vec<String>,
}

impl DeviceInfo {
    /// Human-readable status; codes are interpreted per `kind`.
    pub fn status_text(&self) -> String {
        match (self.kind, self.status) {
            (DeviceKind::OffGrid, Some(n)) => spf_status_text(n).map_or_else(|| format!("Code {n}"), str::to_string),
            (DeviceKind::Meter(_), _) if self.meter.is_some() => "Metering".into(),
            (_, Some(0)) => "Waiting".into(),
            (_, Some(1)) => "Normal".into(),
            (_, Some(3)) => "Fault".into(),
            (_, Some(n)) => format!("Code {n}"),
            (DeviceKind::Unknown, None) => "Unidentified".into(),
            (_, None) => "—".into(),
        }
    }

    /// Coarse health for colouring the status dot.
    pub fn health(&self) -> Health {
        match self.kind {
            DeviceKind::Meter(_) => {
                if self.meter.is_some() {
                    Health::Ok
                } else {
                    Health::Unknown
                }
            }
            DeviceKind::OffGrid => match self.status {
                Some(3) => Health::Fault,
                Some(0 | 4) => Health::Idle, // standby, flashing firmware
                Some(_) => Health::Ok,
                None => Health::Unknown,
            },
            DeviceKind::GridTie | DeviceKind::Hybrid | DeviceKind::Unknown => match self.status {
                Some(1) => Health::Ok,
                Some(3) => Health::Fault,
                Some(_) => Health::Idle,
                None => Health::Unknown,
            },
        }
    }

    /// True when the device reports a different address than the one polled,
    /// which usually means the gateway is not forwarding the unit ID.
    pub fn address_mismatch(&self) -> bool {
        self.com_address.is_some_and(|a| a != self.unit as u16)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Health {
    Ok,
    Idle,
    Fault,
    Unknown,
}

/// SPF/SPE input 0 "System Status", per [OG] V0.26 (codes 0–28).
pub fn spf_status_text(code: u16) -> Option<&'static str> {
    Some(match code {
        0 => "Standby",
        1 => "PV & grid supporting loads",
        2 => "Battery discharging",
        3 => "Fault",
        4 => "Flash",
        5 => "PV charging",
        6 => "Grid charging",
        7 => "PV & grid charging",
        8 => "PV & grid charging + bypass",
        9 => "PV charging + bypass",
        10 => "Grid charging + bypass",
        11 => "Grid bypass",
        12 => "PV charging + loads",
        13 => "PV discharging",
        14 => "PV & battery discharging",
        15 => "Gen charging",
        16 => "Gen charging + bypass",
        17 => "PV & gen charging",
        18 => "PV & gen charging + bypass",
        19 => "PV charging + gen bypass",
        20 => "Gen bypass",
        21 => "PV export",
        22 => "PV export + loads",
        23 => "PV charging + export",
        24 => "PV charging + export + loads",
        25 => "Battery export",
        26 => "Battery export + loads",
        27 => "Battery & PV export",
        28 => "Battery & PV export + loads",
        _ => return None,
    })
}

/// SPH input 1000 work mode, per [P2].
fn sph_work_mode_text(code: u16) -> &'static str {
    match code {
        0 => "waiting",
        1 => "self-test",
        3 => "fault",
        4 => "updating",
        5 => "PV on-grid",
        6 => "battery on-grid",
        7 => "PV+battery off-grid",
        8 => "battery off-grid",
        _ => "?",
    }
}

/// DTC (holding 43) family hint. The table is from [HA] `base.py`, which copies
/// the protocol's DTC list and masks the low byte. It has not been checked
/// against real hardware, so it only orders probes and is never used on its own
/// to decide a kind.
fn dtc_hint(dtc: u16) -> Option<DeviceKind> {
    match dtc & 0xFF00 {
        0x0100..=0x0A00 => Some(DeviceKind::GridTie),
        0x0C00 | 0x1500 => Some(DeviceKind::Hybrid), // "PV storage", "3-phase hybrid"
        0x0D00 => Some(DeviceKind::OffGrid),         // "OffGrid SPF 3-5K"
        _ => None,
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum StorageMap {
    /// SPH/SPA: input 1000..=1014.
    Sph,
    /// TL-XH/MOD-XH/WIT: input 3169..=3181.
    TlXh,
}

/// Probe one slave address.
///
/// * `Ok(Some(_))` – a device answered (including with a normal Modbus
///   exception, which still proves something is listening at that address).
/// * `Ok(None)`    – nothing at that address (timeout or gateway exception).
/// * `Err(_)`      – the gateway itself is unreachable; the scan should stop.
///
/// Request budget (each costs about 1 s on a real bus):
/// * empty ID: 1 (holding 0–124 → 0x0A/0x0B or timeout)
/// * SPF: 2 (holding, input 0–124)
/// * grid-tie / SPH with a 10-char serial: 3 (holding, input, one storage block)
/// * TL-X / MOD-XH with a 30-char serial: 4 (+ serial at 3001)
/// * second storage block: only when the first one misses and the DTC hints at a hybrid
/// * Eastron meter: 3 (holding rejected, input 0x0000, serial 0xFC00)
/// * DTSU666: 4, DDSU666: 5 (holding rejected, SDM read rejected, Chint long/short reads, energy)
/// * unknown device rejecting everything: up to 7 (holding, SDM, 2× Chint, 3 identity pieces);
///   the SDM per-block fallback adds 2 only when the long SDM read fails with
///   something other than exception 0x01/0x02
pub fn probe(client: &mut ModbusTcp, unit: u8) -> Result<Option<DeviceInfo>, ModbusError> {
    let mut info = DeviceInfo { unit, ..Default::default() };

    // Presence check doubles as the full base-map read: one request instead of four.
    let mut holding_exc = None;
    match client.read(unit, Table::Holding, 0, 125) {
        Ok(h) => {
            info.notes.push("holding 0–124: ok".into());
            apply_holding(&mut info, |a| h.get(a as usize).copied());
            info.device_type_code = Some(h[43]);
            info.notes.push(format!("DTC {} (0x{:04X}), holding 44 = 0x{:04X}", h[43], h[43], h[44]));
            if !has_growatt_identity(&info) {
                info.notes.push(format!("holding non-zero: {}", nonzero_summary(&h, 0, 10)));
            }
        }
        Err(e @ ModbusError::Connect(_)) => return Err(e),
        Err(e) if e.is_gateway_no_response() => return Ok(None),
        Err(ModbusError::Exception(c)) => {
            info.notes.push(format!("holding 0–124: exception 0x{c:02X}"));
            holding_exc = Some(c);
        }
        Err(_) => return Ok(None),
    }

    let mut identified = has_growatt_identity(&info);
    if !identified {
        // A meter rejects the Growatt block (or returns data with no Growatt identity).
        if let Some(m) = meter::probe(client, unit, &mut info.notes)? {
            info.kind = DeviceKind::Meter(m.kind);
            info.meter = Some(m.info);
            info.serial = m.serial;
            return Ok(Some(info));
        }
        if let Some(code) = holding_exc {
            // Something is there but rejected the big block. Read the identity
            // pieces individually, unless FC 0x03 is unsupported altogether.
            if code != 0x01 {
                let mut regs = std::collections::HashMap::new();
                for (start, count) in [(9u16, 3u16), (22, 6), (30, 1)] {
                    match client.read(unit, Table::Holding, start, count) {
                        Ok(r) => {
                            info.notes.push(format!("holding {start}×{count}: {}", nonzero_summary(&r, start, 6)));
                            regs.extend((start..).zip(r));
                        }
                        Err(e @ ModbusError::Connect(_)) => return Err(e),
                        Err(e) => info.notes.push(format!("holding {start}×{count}: {}", meter::short_err(&e))),
                    }
                }
                apply_holding(&mut info, |a| regs.get(&a).copied());
            }
            identified = has_growatt_identity(&info);
            if !identified {
                info.kind = DeviceKind::Unknown;
                return Ok(Some(info));
            }
        }
    }

    probe_inverter(client, unit, &mut info, identified)?;
    Ok(Some(info))
}

/// Growatt path: serial fallback, input map, SPF signature, storage block.
fn probe_inverter(client: &mut ModbusTcp, unit: u8, info: &mut DeviceInfo, identified: bool) -> Result<(), ModbusError> {
    let hint = info.device_type_code.and_then(dtc_hint);

    let mut long_serial = false;
    if info.serial.is_none() {
        for start in [3001u16, 133] {
            match client.read(unit, Table::Holding, start, 15) {
                Ok(r) => {
                    info.serial = non_empty(ascii(&r));
                    if info.serial.is_some() {
                        info.notes.push(format!("serial at holding {start}–{}", start + 14));
                        long_serial = start == 3001;
                        break;
                    }
                }
                Err(e @ ModbusError::Connect(_)) => return Err(e),
                Err(e) => info.notes.push(format!("holding {start}×15: {}", meter::short_err(&e))),
            }
        }
    }

    let mut input = None;
    for count in [125u16, 57] {
        match client.read(unit, Table::Input, 0, count) {
            Ok(r) => {
                info.notes.push(format!("input 0–{}: ok", count - 1));
                input = Some(r);
                break;
            }
            Err(e @ ModbusError::Connect(_)) => return Err(e),
            Err(e) => info.notes.push(format!("input 0–{}: {}", count - 1, meter::short_err(&e))),
        }
    }
    let Some(r) = input else {
        info.kind = if identified { DeviceKind::GridTie } else { DeviceKind::Unknown };
        return Ok(());
    };

    if looks_like_spf(&r, hint) {
        info.kind = DeviceKind::OffGrid;
        info.notes.push("SPF input layout (battery V at 17, SOC at 18)".into());
        apply_spf_input(info, &r);
        return Ok(());
    }

    info.status = Some(r[0]);
    info.pv_power_w = Some(u32_at(&r, 1) as f64 / 10.0);
    info.ac_power_w = Some(u32_at(&r, 35) as f64 / 10.0);
    info.energy_today_kwh = Some(u32_at(&r, 53) as f64 / 10.0);
    info.energy_total_kwh = Some(u32_at(&r, 55) as f64 / 10.0);
    let grid_hz_ok = (4500..=6500).contains(&r[37]);

    // One storage probe per inverter. The 30-char serial at 3001 marks the
    // TL-X/TL-XH (and WIT) generation; anything else tries the SPH/SPA block.
    let first = if long_serial { StorageMap::TlXh } else { StorageMap::Sph };
    let mut battery = read_storage(client, unit, first, info)?;
    if battery.is_none() && hint == Some(DeviceKind::Hybrid) {
        let other = if first == StorageMap::Sph { StorageMap::TlXh } else { StorageMap::Sph };
        battery = read_storage(client, unit, other, info)?;
    }

    info.kind = if battery.is_some() {
        DeviceKind::Hybrid
    } else if identified || grid_hz_ok {
        DeviceKind::GridTie
    } else {
        DeviceKind::Unknown
    };
    info.battery = battery;
    Ok(())
}

fn read_storage(
    client: &mut ModbusTcp,
    unit: u8,
    map: StorageMap,
    info: &mut DeviceInfo,
) -> Result<Option<BatteryInfo>, ModbusError> {
    let (start, count, what) = match map {
        StorageMap::Sph => (1000u16, 15u16, "input 1000–1014 (SPH storage)"),
        StorageMap::TlXh => (3169, 13, "input 3169–3181 (TL-XH storage)"),
    };
    let r = match client.read(unit, Table::Input, start, count) {
        Ok(r) => r,
        Err(e @ ModbusError::Connect(_)) => return Err(e),
        Err(e) => {
            info.notes.push(format!("{what}: {}", meter::short_err(&e)));
            return Ok(None);
        }
    };
    let (v, soc, dis, chg) = match map {
        StorageMap::Sph => (r[13] as f64 / 10.0, r[14], u32_at(&r, 9), u32_at(&r, 11)),
        StorageMap::TlXh => (r[0] as f64 / 100.0, r[2], u32_at(&r, 9), u32_at(&r, 11)),
    };
    // A block of zeros (no battery) or an out-of-range SOC is not a storage map.
    if soc > 100 || (soc == 0 && v == 0.0) {
        info.notes.push(format!("{what}: ok but no battery data (SOC {soc}, V {v:.1})"));
        return Ok(None);
    }
    info.notes.push(format!("{what}: SOC {soc} %"));
    if map == StorageMap::Sph {
        info.notes.push(format!("SPH work mode {} ({})", r[0], sph_work_mode_text(r[0])));
    }
    Ok(Some(BatteryInfo {
        soc_pct: Some(soc as f64),
        voltage_v: Some(v),
        charge_w: Some(chg as f64 / 10.0),
        discharge_w: Some(dis as f64 / 10.0),
    }))
}

/// SPF signature on input 0..: battery voltage at 17 (0.01 V, 9–65 V systems)
/// and SOC at 18, plus a mains frequency at 21 (line) or 23 (output).
/// On the grid-tie map 17..=18 and 21..=22 are PV4/PV5 power (u32), whose high
/// words are 0 in practice, and 37 is the grid frequency.
fn looks_like_spf(r: &[u16], hint: Option<DeviceKind>) -> bool {
    if r.len() < 38 {
        return false;
    }
    let hz = |x: u16| (4500..=6500).contains(&x);
    let battery = (900..=6500).contains(&r[17]) && r[18] <= 100;
    let freq = hz(r[21]) || (hz(r[23]) && !hz(r[37]));
    r[0] <= 30 && battery && (freq || hint == Some(DeviceKind::OffGrid))
}

fn apply_spf_input(info: &mut DeviceInfo, r: &[u16]) {
    info.status = Some(r[0]);
    info.pv_power_w = Some((u32_at(r, 3) + u32_at(r, 5)) as f64 / 10.0);
    info.ac_power_w = Some(u32_at(r, 9) as f64 / 10.0);
    if r.len() >= 56 {
        info.energy_today_kwh = Some((u32_at(r, 48) + u32_at(r, 52)) as f64 / 10.0);
        info.energy_total_kwh = Some((u32_at(r, 50) + u32_at(r, 54)) as f64 / 10.0);
    }
    let bat_w = (r.len() >= 79).then(|| u32_at(r, 77) as i32 as f64 / 10.0); // + = discharge
    info.battery = Some(BatteryInfo {
        soc_pct: Some(r[18] as f64),
        voltage_v: Some(r[17] as f64 / 100.0),
        charge_w: bat_w.map(|w| (-w).max(0.0)),
        discharge_w: bat_w.map(|w| w.max(0.0)),
    });
}

/// Growatt identity: an ASCII serial or firmware string in the base map.
fn has_growatt_identity(info: &DeviceInfo) -> bool {
    info.serial.is_some() || info.firmware.is_some()
}

/// "0=1, 3=100, …" for the first `max` non-zero registers.
fn nonzero_summary(regs: &[u16], base: u16, max: usize) -> String {
    let parts: Vec<String> = regs
        .iter()
        .enumerate()
        .filter(|(_, v)| **v != 0)
        .take(max)
        .map(|(i, v)| format!("{}={v}", base as usize + i))
        .collect();
    if parts.is_empty() { "all zero".into() } else { parts.join(", ") }
}

fn apply_holding(info: &mut DeviceInfo, reg: impl Fn(u16) -> Option<u16>) {
    let block = |start: u16, n: u16| -> Option<Vec<u16>> { (start..start + n).map(&reg).collect() };
    info.firmware = block(9, 3).and_then(|r| non_empty(ascii(&r)));
    info.serial = block(23, 5).and_then(|r| non_empty(ascii(&r)));
    info.baud = reg(22).and_then(|b| match b {
        0 => Some(9600),
        1 => Some(38400),
        _ => None,
    });
    info.com_address = reg(30).filter(|a| (1..=254).contains(a));
}

fn u32_at(regs: &[u16], i: usize) -> u32 {
    ((regs[i] as u32) << 16) | regs[i + 1] as u32
}

fn ascii(regs: &[u16]) -> String {
    regs.iter()
        .flat_map(|r| r.to_be_bytes())
        .filter(|b| b.is_ascii_graphic())
        .map(|b| b as char)
        .collect()
}

fn non_empty(s: String) -> Option<String> {
    Some(s).filter(|s| !s.is_empty())
}
